#![no_std]
#![allow(non_snake_case)] // I want to keep with the manufacturers naming scheme.

use defmt::{debug, error, info, trace, Format};

use embassy_rp::dma::Channel;
use embassy_rp::gpio::{AnyPin, Input, Level, Pull}; // For the wakeup.
use embassy_rp::interrupt::typelevel::Binding;
use embassy_rp::uart::{
    Blocking, Config, Instance, InterruptHandler, RxPin, TxPin, Uart, UartRx, UartTx,
};
use embassy_rp::Peri;
use embassy_time::Timer;

use core::mem::transmute;
use heapless::Vec;

// =====

const REPLY_DATA_SIZE: usize = 1024; // Some commands *really* return some large data sets!! :)

#[derive(Copy, Clone, Format, PartialEq)]
#[repr(u8)]
pub enum Status {
    CmdExecComplete = 0x00,
    ErrorReceivePackage = 0x01,
    ErrorNoFingerOnSensor = 0x02,
    ErrorEnroleFinger = 0x03,
    ErrorGenCharFileDistortedImage = 0x06,
    ErrorGenCharFileSmallImage = 0x07,
    ErrorNoFingerMatch = 0x08,
    ErrorNoMatchingFinger = 0x09,
    ErrorCombineCharFiles = 0x0a,
    ErrorPageIdBeyondLibrary = 0x0b,
    ErrorReadingTemplateFromLibrary = 0x0c,
    ErrorUploadTemplate = 0x0d,
    ErrorReceiveData = 0x0e,
    ErrorUploadImage = 0x0f,
    ErrorDeleteTemplate = 0x10,
    ErrorClearLibrary = 0x11,
    ErrorPassword = 0x13,
    ErrorMissingValidPrimaryImage = 0x15,
    ErrorWriteFlash = 0x18,
    ErrorNoDef = 0x19,
    ErrorInvalidRegister = 0x1a,
    ErrorIncorrectConfigRegister = 0x1b,
    ErrorWrongNotepadNumber = 0x1c,
    ErrorFailedOperateCommunicationPort = 0x1d,
    ErrorSensorAbnormal = 0x29,
    ErrorBadPackage = 0xff,
}

// https://www.reddit.com/r/rust/comments/36pgn9/integer_to_enum_after_removal_of_fromprimitive/
impl From<u8> for Status {
    fn from(t: u8) -> Status {
        unsafe { transmute(t) }
    }
}

// These are in Hex order. Further down, they're defined in the order they
// came in the documentation.
#[derive(Copy, Clone, PartialEq)]
#[repr(u8)]
pub enum Command {
    GenImg = 0x01,
    Img2Tz = 0x02,
    Match = 0x03,
    Search = 0x04,
    RegModel = 0x05,
    Store = 0x06,
    LoadChar = 0x07,
    UpChar = 0x08,
    DownChar = 0x09,
    UpImage = 0x0a,
    DownImage = 0x0b,
    DeletChar = 0x0c,
    Empty = 0x0d,
    SetSysPara = 0x0e,
    ReadSysPara = 0x0f,
    SetPwd = 0x12,
    VfyPwd = 0x13,
    SetAdder = 0x15,
    ReadInfPage = 0x16,
    Control = 0x17,
    ReadNotepad = 0x19,
    TempleteNum = 0x1d,
    GetImageEx = 0x28,
    Cancel = 0x30,
    CheckSensor = 0x36,
    GetAlgVer = 0x39,
    GetFwVer = 0x3a,
    SoftRst = 0x3d,
    HandShake = 0x40,
    GetRandomCode = 0x14,
    WriteNotepad = 0x18,
    ReadIndexTable = 0x1f,
    AuraLedConfig = 0x35,
    ReadProdInfo = 0x3c,
}

#[derive(Copy, Clone)]
#[repr(u16)]
pub enum Packets {
    StartCode = 0xEF01, // High byte transferred first.
    CommandPacket = 0x01,
    DataPacket = 0x02,
    AckPacket = 0x07,
    EndDataPacket = 0x08,
}

#[derive(Copy, Clone)]
#[repr(u8)]
pub enum PacketCode {
    CommandPacket = 0x01,
    DataPacket = 0x02,
    AckPacket = 0x07,
    DataPackageEnd = 0x08,
}

#[derive(Copy, Clone)]
#[repr(u8)]
pub enum AuroraLEDControl {
    BreathingLight = 0x01,
    FlashingLight = 0x02,
    AlwaysOn = 0x03,
    AlwaysOff = 0x04,
    GraduallyOn = 0x05,
    GraduallyOff = 0x06,
}

#[derive(Copy, Clone)]
#[repr(u8)]
pub enum AuroraLEDColour {
    Red = 0x01,
    Blue = 0x02,
    Purple = 0x03,
}

// Highly subjective, but..
#[derive(Copy, Clone)]
#[repr(u8)]
pub enum AuroraLEDSpeed {
    Slow = 0xC8,
    Medium = 0x20,
    Fast = 0x02,
}

#[derive(Copy, Clone)]
#[repr(u8)]
pub enum Parameters {
    BaudRate = 0x04,
    SecurityLevel = 0x05,
    DataPkgLen = 0x06,
}

#[derive(Copy, Clone, Format)]
#[repr(u16)]
pub enum SecurityLevels {
    One = 1,
    Two = 2,
    Three = 3,
    Four = 4,
    Five = 5,
}

impl From<u16> for SecurityLevels {
    fn from(t: u16) -> SecurityLevels {
        assert!(Self::One as u16 <= t && t <= Self::Five as u16);
        unsafe { transmute(t) }
    }
}

// =====

struct SystemParameters {
    status_register: u16,
    system_id: u16,
    library_size: u16,
    security_level: u16,
    device_address: u32,
    data_size: u16,
    baud_rate: u16,
}

struct ProductInfo {
    fpm_model: u128,
    batch_nr: u32,
    serial_nr: u64,
    hw_nr: u16,
    fps_model: u64,
    fps_width: u16,
    fps_height: u16,
    tmpl_size: u16,
    tmpl_total: u16,
}

pub struct R503<'l> {
    tx: UartTx<'l, Blocking>,
    rx: UartRx<'l, Blocking>,
    wakeup: Input<'l>,

    pub address: u32,
    pub password: u32,
    buffer: Vec<u8, REPLY_DATA_SIZE>,
    params: SystemParameters,
    prodinfo: ProductInfo,
    templatenum: u16
}

// Channel => DMA_CH0/DMA_CH1
impl<'l> R503<'l> {
    //! Data package format
    //! | Name    | Length  | Description
    //! | :--     | --:     | :--
    //! | Start   | 2 bytes | Fixed value of 0xEF01. High byte transferred first.
    //! | Address | 4 bytes | Default value is 0xFFFFFFFF, which can be modified by command.<br>High byte transferred first and at wrong adder value, module will reject to transfer.
    //! | PID     | 1 byte  | 01H Command packet<br>02H Data packet. Data packet shall not appear alone in executing processs, must follow command packet or acknowledge packet.<br>07H Acknowledge packet<br>08H End of Data packet.
    //! | LENGTH  | 2 bytes | Refers to the length of package content (command packets and data packets) plus the length of Checksum (2 bytes). Unit is byte. Max length is 256 bytes.<br>High byte is transferred first.
    //! | DATA    | -       | It can be commands, data, command’s parameters, acknowledge result, etc.<br>(fingerprint character value, template are all deemed as data).
    //! | SUM     | 2 bytes | The arithmetic sum of package identifier, package length and all package contens. Overflowing bits are omitted. High byte is transferred first.
    //!
    //! * Header: Start + Address (6 bytes).
    //! * Package: Package ID + Package Length + Data (3 + x bytes).
    //! * Length: Calculated on Package ID and Data.
    //!   * Actually, full length, is:
    //!     + Start
    //!     + Address
    //!     + Package Length (Package ID + Data)
    //!     + Checksum
    //!     = 11 bytes (plus length of data).
    //! * Checksum: Calculated on Package ID + Package Length + Data.<br>
    //! **NOTE**: Pins must be consecutive, otherwise it'll segfault!
    pub fn new<T: Instance>(
        uart: Peri<'l, T>,
        _irqs: impl Binding<<T as embassy_rp::uart::Instance>::Interrupt, InterruptHandler<T>>,
        pin_send: Peri<'l, impl TxPin<T>>,
        _pin_send_dma: Peri<'l, impl Channel>,
        pin_receive: Peri<'l, impl RxPin<T>>,
        _pin_receive_dma: Peri<'l, impl Channel>,
        pin_wakeup: Peri<'l, AnyPin>,
    ) -> Self {
        // Set default passwords.
        let address = 0xFFFFFFFF;
        let password = 0x00000000;

        // Configure the communication protocol etc.
        let mut config = Config::default(); // => 115200/8N1
        config.baudrate = 57600;

        // Initialize the fingerprint scanner.
        let uart = Uart::new_blocking(
            uart,
            pin_send,
            pin_receive,
            // irqs,
            // pin_send_dma,
            // pin_receive_dma,
            config,
        );
        let (tx, rx) = uart.split();

        // Initialize the WAKEUP pin.
        let wakeup = Input::new(pin_wakeup, Pull::Down);
        match wakeup.get_level() {
            Level::Low => debug!("Initial FP WAKEUP pin level: LOW"),
            Level::High => debug!("Initial FP WAKEUP pin level: HIGH"),
        }

        Self {
            tx: tx,
            rx: rx,
            wakeup: wakeup,

            address: address,
            password: password,

            buffer: heapless::Vec::new(),

            params: SystemParameters {
                status_register: 0,
                system_id: 0,
                library_size: 0,
                security_level: 0,
                device_address: 0,
                data_size: 0,
                baud_rate: 0,
            },

            prodinfo: ProductInfo {
                fpm_model: 0,
                batch_nr: 0,
                serial_nr: 0,
                hw_nr: 0,
                fps_model: 0,
                fps_width: 0,
                fps_height: 0,
                tmpl_size: 0,
                tmpl_total: 0,
            },

            templatenum: 0
        }
    }

    // ===== Internal functions

    /// Low level write - write the data in `self.buffer`, return `Status`.
    async fn write(&mut self) -> Status {
        debug!("write='{:?}'", self.buffer[..]);
        let _ = self.debug_vec(&self.buffer, true).await;

        match self.tx.blocking_write(&self.buffer) {
            Ok(..) => {
                debug!("Write successful");
                return Status::CmdExecComplete;
            }
            Err(e) => {
                error!("Write error: {:?}", e);
                return Status::ErrorWriteFlash;
            }
        }
    }

    /// Low level read - read one byte, and return it as is.
    async fn read(&mut self) -> [u8; 1] {
        let mut buf: [u8; 1] = [0; 1]; // Can only read one byte at a time!

        match self.rx.blocking_read(&mut buf) {
            Ok(_) => {
                // Extract and save read byte.
                return buf;
            }
            Err(_) => return [0; 1], // TimeoutError -> Ignore.
        }
    }

    /// Read the full reply from the device.
    async fn read_reply(&mut self, command: Command) -> Vec<u8, REPLY_DATA_SIZE> {
        debug!("Reading reply");

        let mut data: Vec<u8, REPLY_DATA_SIZE> = heapless::Vec::new(); // Return buffer.

        // Read *at least* 9 bytes - just so we can get the actual length of the package.
        let mut pkg_len: u16 = 9;
        let mut cnt: u16 = 0; // Keep track of how many packages we've received.

        // Clear buffer before we populate it with the response.
        self.buffer.clear();

        // Loop until we can get the package length (byte 8 and 9).
        loop {
            if cnt == 9 {
                pkg_len = 6 + u16::from_be_bytes(data[7..9].try_into().unwrap()) + 2;
                trace!(
                    "  Actual package length: {} (2+4+{}+{}+2={}; read={})",
                    pkg_len,
                    data[7],
                    data[8],
                    pkg_len,
                    u16::from_be_bytes(data[7..9].try_into().unwrap())
                );
            }

            // Want to make sure we read at least the last one.
            if cnt > pkg_len {
                break;
            }

            // Read byte.
            let buf: [u8; 1] = self.read().await;
            trace!("  r({:03})='{=u8:#04x}H' ({:03}D)", cnt, buf[0], buf[0]);
            let _ = data.push(buf[0]).unwrap();

            cnt = cnt + 1;
        }
        debug!("read='{:?}'", data[..]);

        if data.len() < 1 {
            error!("Empty response - no data");
            return data;
        }

        // The SoftRst command will, after successful package send, send `0x55H` as a handshake.
        // So we need to read ONE more byte here before we leave.
        if command == Command::SoftRst {
            let buf: [u8; 1] = self.read().await;
            trace!(
                "  R({:03})='{=u8:#04x}H' ({:03}D) - SoftRst response",
                cnt,
                buf[0],
                buf[0]
            );

            if buf[0] == 0x55 {
                trace!("  Successful reset detected");
            } else {
                error!("Unsuccessful reset detected");
            }
        }

        // Save the response.
        debug!("Read successful");
        self.buffer = data.clone();

        return data;
    }

    /// Format a command package and write it to the device with `self.write`.
    async fn send_command(&mut self, command: Command, data: Vec<u8, REPLY_DATA_SIZE>) -> Status {
        debug!(
            "Sending command {=u8:#04x}H ({:?})",
            command as u8,
            self.debug_vec(&data, false).await
        );

        // Clear buffer before we populate it with the command data.
        self.buffer.clear();

        // Setup data package.
        self.write_cmd_bytes(&(Packets::StartCode as u16).to_be_bytes()[..])
            .await;
        self.write_cmd_bytes(&self.address.to_be_bytes()[..]).await;
        self.write_cmd_bytes(&[PacketCode::CommandPacket as u8])
            .await;

        // Add the length of the package content (command packets and data packets). See below.
        // Length is calculated on 'the Package Identifier (1 byte) + data (??) + checksum (2 bytes)'.
        let len: u16 = (1 + data.len() + 2).try_into().unwrap();
        self.write_cmd_bytes(&len.to_be_bytes()[..]).await; // Package Length	u16

        // Add the instruction code (command).
        self.write_cmd_bytes(&[command as u8]).await; // Instruction Code	u8

        // Add the data, if any.
        self.write_cmd_bytes(&data).await;

        // Calculate and add checksum.
        let chk = self.compute_checksum(self.buffer.len()).await;
        self.write_cmd_bytes(&chk.to_be_bytes()[..]).await; // Checksum

        // Send package.
        match self.write().await {
            Status::ErrorWriteFlash => {
                error!("Write failed");
                return Status::ErrorWriteFlash;
            }
            _ => {
                // Fall through..
            }
        }
        Timer::after_millis(250).await; // Give it 1/4s to settle.

        // Read response.
        if self.read_reply(command).await.is_empty() {
            error!("Read returned empty data");
            return Status::ErrorReceivePackage;
        };

        // Parse result and return the Status.
        return self.parse_result(command).await;
    }

    async fn write_cmd_bytes(&mut self, bytes: &[u8]) {
        let _ = self.buffer.extend_from_slice(bytes);
    }

    /// We calculate the checksum on Package ID, Package Length and Data.
    /// This is always starting at byte 6. How far we read in the buffer
    /// depends on if it's a write or a read - the read already HAVE the
    /// checksum in it..
    async fn compute_checksum(&self, check_end: usize) -> u16 {
        let mut checksum: u16 = 0;
        let checked_bytes = &self.buffer[6..check_end];

        for byte in checked_bytes {
            checksum += (*byte) as u16;
        }

        return checksum;
    }

    /// Parse the result of the response from the device.
    async fn parse_result(&mut self, command: Command) -> Status {
        debug!(
            "Parsing reply ({:?})",
            self.debug_vec(&self.buffer, false).await
        );

        if self.buffer.is_empty() {
            return Status::ErrorReceivePackage;
        } else if self.buffer[9] != Status::CmdExecComplete as u8 {
            debug!(
                "Command did not complete: {:?}",
                Status::from(self.buffer[9])
            );
            return Status::from(self.buffer[9]);
        }

        // ----
        // START
        let start = u16::from_be_bytes(self.buffer[0..2].try_into().unwrap());
        if start != Packets::StartCode as u16 {
            error!("Bad package (start)");
            return Status::ErrorReceivePackage;
        } else {
            debug!("  Package start is ok");
        }

        // ----
        // ADDRESS (new)
        let address = u32::from_be_bytes(self.buffer[2..6].try_into().unwrap());

        if command == Command::SetAdder && address != self.address {
            // Change of address was requested
            // AND the scanner reported all ok
            // AND the address returned does not match the one we used initially.
            // => Update the global address.
            self.address = address;
            info!("Address updated");
        } else {
            if address != self.address {
                error!("Bad package (address)");
                return Status::ErrorReceivePackage;
            } else {
                debug!("  Package address is ok");
            }
        }

        // ----
        // PACKAGE ID
        let pid = u8::from_be_bytes([self.buffer[6]]);
        if pid != PacketCode::AckPacket as u8 {
            error!("Bad package (pid)");
            return Status::ErrorReceivePackage;
        } else {
            debug!("  Package pid is ok");
        }

        // ----
        // PACKAGE LENGTH
        // => `pid` (1 byte) + `data` (x bytes) + `checksum` (2 bytes).
        let package_len = u16::from_be_bytes(self.buffer[7..9].try_into().unwrap());
        debug!("  Package length: {}", package_len);

        // ----
        // DATA
        let data_length: u16 = package_len - 3;
        debug!("  Data length: {}", data_length);
        if data_length > 0 {
            // This is where it gets a bit ugly..

            if command == Command::ReadSysPara {
                debug!("  Checking data package for ReadSysPara");
                self.params = SystemParameters {
                    status_register: u16::from_be_bytes(self.buffer[10..12].try_into().unwrap()),
                    system_id: u16::from_be_bytes(self.buffer[12..14].try_into().unwrap()),
                    library_size: u16::from_be_bytes(self.buffer[14..16].try_into().unwrap()),
                    security_level: u16::from_be_bytes(self.buffer[16..18].try_into().unwrap()),
                    device_address: u32::from_be_bytes(self.buffer[18..22].try_into().unwrap()),
                    data_size: u16::from_be_bytes(self.buffer[22..24].try_into().unwrap()),
                    baud_rate: u16::from_be_bytes(self.buffer[24..26].try_into().unwrap())
                };

                trace!("  System parameters:");
                trace!(
                    "    Status register: {=u16:#06x}",
                    self.params.status_register
                );
                trace!("    System ID: {=u16:#06x}", self.params.system_id);
                trace!("    Library size: {:03}", self.params.library_size);
                trace!(
                    "    Security level: {}",
                    SecurityLevels::from(self.params.security_level)
                );
                trace!(
                    "    Device address: {=u32:#010x}",
                    self.params.device_address
                );
                trace!(
                    "    Max data package length: {}",
                    self.translate_data_package_size(self.params.data_size)
                );
                trace!("    Baud rate: {}", self.params.baud_rate * 9600); // Value in increments of 9600.
            } else if command == Command::ReadProdInfo {
                debug!("  Checking product information for ReadProdInfo");
                self.prodinfo = ProductInfo {
                    fpm_model: u128::from_be_bytes(self.buffer[10..26].try_into().unwrap()),
                    batch_nr: u32::from_be_bytes(self.buffer[26..30].try_into().unwrap()),
                    serial_nr: u64::from_be_bytes(self.buffer[30..38].try_into().unwrap()),
                    hw_nr: u16::from_be_bytes(self.buffer[38..40].try_into().unwrap()),
                    fps_model: u64::from_be_bytes(self.buffer[40..48].try_into().unwrap()),
                    fps_width: u16::from_be_bytes(self.buffer[48..50].try_into().unwrap()),
                    fps_height: u16::from_be_bytes(self.buffer[50..52].try_into().unwrap()),
                    tmpl_size: u16::from_be_bytes(self.buffer[52..53].try_into().unwrap()),
                    tmpl_total: u16::from_be_bytes(self.buffer[54..56].try_into().unwrap())
                };

                trace!("  Product information:");
                trace!("    FPM Model: {=u128:x}", self.prodinfo.fpm_model);
                trace!("    Batch nr: {=u32:x}", self.prodinfo.batch_nr);
                trace!("    Serial nr: {=u64:x}", self.prodinfo.serial_nr);
                trace!("    Hardware nr: {=u16:x}", self.prodinfo.hw_nr);
                trace!("    FPS Model: {=u64:x}", self.prodinfo.fps_model);
                trace!("    FPS Width: {=u16:x}", self.prodinfo.fps_width);
                trace!("    FPS Height: {=u16:x}", self.prodinfo.fps_height);
                trace!("    Template size: {=u16:x}", self.prodinfo.tmpl_size);
                trace!("    Templates total: {=u16:x}", self.prodinfo.tmpl_total);
            } else if command == Command::TempleteNum {
                debug!("  Checking template number for TempleteNum");
                self.templatenum = u16::from_be_bytes(self.buffer[10..12].try_into().unwrap());
                trace!("  Number of templates: {=u16:x}", self.templatenum);
            }
        }

        // ----
        // CHECKSUM
        // Retrieve and calculate the checksum.
        // Header (2 bytes) + Address (4 bytes)
        let len = self.buffer.len();
        let rchk: u16 = u16::from_be_bytes([self.buffer[len - 2], self.buffer[len - 1]]);
        let cchk = self.compute_checksum(len - 2).await;
        trace!(
            "  Package length: {}(read={}), Read checksum: {}; Computed checksum: {}",
            len,
            package_len,
            rchk,
            cchk
        );

        // ----
        // Return success or fail.
        if rchk == cchk {
            debug!("  Checksum is ok");
            return Status::CmdExecComplete;
        } else {
            error!("Checksums don't match");
            return Status::ErrorBadPackage;
        }
    }

    // Go through our Vec and output it, if or when needed.
    async fn debug_vec(&self, buf: &Vec<u8, REPLY_DATA_SIZE>, out: bool) -> [u8; REPLY_DATA_SIZE] {
        let mut a: [u8; REPLY_DATA_SIZE] = [0; REPLY_DATA_SIZE];
        let mut i = 0;

        for x in buf {
            if out {
                trace!("  x({:03})='{=u8:#04x}H' ({:03}D)", i, x, x);
            }
            a[i] = *x;
            i = i + 1;
        }

        return a;
    }

    // Translate the system parameters max data package length to string.
    // TODO: Would be nice with an enum or something here..
    fn translate_data_package_size(&self, size: u16) -> &'static str {
        let val: &str;
        match size {
            0 => val = "32bytes/package",
            1 => val = "64bytes/package",
            2 => val = "128bytes/package",
            3 => val = "256bytes/package",
            _ => panic!("Unknown value: {}", size),
        }
        return val;
    }

    // ===== System-related instructions

    /// # Description
    /// Verify Module’s handshaking password.
    /// # Input Parameter: (4 bytes)
    /// * PassWord (4 bytes)
    /// # Return Parameter
    /// * Confirmation code: (1 byte)
    ///   - Confirmation code = 00H: Correct password;
    ///   - Confirmation code = 01H: Error when receiving package;
    ///   - Confirmation code = 13H: Wrong password;
    /// # Instruction code
    /// 13H
    /// # Command Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x01
    /// * Package Length          2 byte         0x0007
    /// * Instruction code        1 byte         0x13
    /// * Data                    4 bytes
    ///   - PassWord              4 byte
    /// * Checksum                2 bytes        Sum                (see top)
    /// # Acknowledge Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x07
    /// * Package Length          2 byte         0x0003
    /// * Confirmation code       1 byte         xx                 (see above)
    /// * Checksum                2 bytes        Sum                (see top)
    pub async fn VfyPwd(&mut self, pass: u32) -> Status {
        info!("COMMAND: Checking password: {=u32:#010x}H", pass);

        let mut data: Vec<u8, REPLY_DATA_SIZE> = heapless::Vec::new();
        let split: [u8; 4] = pass.to_be_bytes();
        data.extend(split.iter().map(|&i| i));

        return self.send_command(Command::VfyPwd, data).await;
    }

    /// # Description
    /// Set Module’s handshaking password.
    /// # Input Parameter: (4 bytes)
    /// * PassWord (4 bytes)
    /// # Return Parameter
    /// * Confirmation code: (1 byte)
    ///   - Confirmation code=00H: password setting complete;
    ///   - Confirmation code=01H: error when receiving package;
    /// # Instruction code
    /// 12H
    /// # Command Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x01
    /// * Package Length          2 byte         0x0007
    /// * Instruction code        1 byte         0x12
    /// * Data                    4 bytes
    ///   - PassWord              4 byte
    /// * Checksum                2 bytes        Sum                (see top)
    /// # Acknowledge Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x07
    /// * Package Length          2 byte         0x0003
    /// * Confirmation code       1 byte         xx                 (see above)
    /// * Checksum                2 bytes        Sum                (see top)
    pub async fn SetPwd(&mut self, pass: u32) -> Status {
        info!("COMMAND: Setting module password: {=u32:#010x}H", pass);

        let mut data: Vec<u8, REPLY_DATA_SIZE> = heapless::Vec::new();
        let split: [u8; 4] = pass.to_be_bytes();
        data.extend(split.iter().map(|&i| i));

        match self.send_command(Command::SetPwd, data).await {
            Status::CmdExecComplete => {
                info!("Password changed");
                self.password = pass;
                return Status::CmdExecComplete;
            }
            ret @ _ => return ret,
        }
    }

    /// # Description
    /// Set Module address.
    /// # Input Parameter
    /// None.
    /// # Return Parameter
    /// * Confirmation code: (1 byte)
    ///   - Confirmation code=00H: address setting complete;
    ///   - Confirmation code=01H: error when receiving package;
    /// # Instruction code
    /// 15H
    /// # Command Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x01
    /// * Package Length          2 byte         0x0007
    /// * Instruction code        1 byte         0x13
    /// * Data                    4 bytes
    ///   - NewAddress            4 byte
    /// * Checksum                2 bytes        Sum                (see top)
    /// # Acknowledge Package format
    /// * Header                  2 bytes        0xEF01
    /// * NewAddress              4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x07
    /// * Package Length          2 byte         0x0007
    /// * Confirmation code       1 byte         xx                 (see above)
    /// * Checksum                2 bytes        Sum                (see top)
    pub async fn SetAdder(&mut self, addr: u32) -> Status {
        info!("COMMAND: Setting module address: {=u32:#010x}", addr);

        let mut data: Vec<u8, REPLY_DATA_SIZE> = heapless::Vec::new();
        let split: [u8; 4] = addr.to_be_bytes();
        data.extend(split.iter().map(|&i| i));

        return self.send_command(Command::SetAdder, data).await;
    }

    /// # Description
    /// Operation parameter settings.
    /// # Input Parameter (2 bytes)
    /// * Parameter number (1 + 1 byte).
    /// # Return Parameter
    /// * Confirmation code: (1 byte)
    ///   - Confirmation code=00H: parameter setting complete;
    ///   - Confirmation code=01H: error when receiving package;
    ///   - Confirmation code=1aH: wrong register number;
    /// # Instruction code
    /// 0eH
    /// # Command Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x01
    /// * Package Length          2 byte         0x0007
    /// * Instruction code        1 byte         0x0e
    /// * Data                    2 bytes
    ///   - Parameter Number      1 byte         4/5/6
    ///   - Content               1 byte         xx
    /// * Checksum                2 bytes        Sum                (see top)
    /// # Acknowledge Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x07
    /// * Package Length          2 byte         0x0003
    /// * Confirmation code       1 byte         xx                 (see above)
    /// * Checksum                2 bytes        Sum                (see top)
    pub async fn SetSysPara(&mut self, param: u8, content: u8) -> Status {
        info!(
            "COMMAND: Set system parameters: {=u8:#04x}H/{=u8:#04x}H",
            param, content
        );

        let mut data: Vec<u8, REPLY_DATA_SIZE> = heapless::Vec::new();
        let _ = data.push(param);
        let _ = data.push(content);

        return self.send_command(Command::SetSysPara, data).await;
    }

    /// # Description
    /// * For UART protocol, it control the “on/off” of USB port;
    /// * For USB protocol, it control the “on/off” of UART port;
    /// # Input Parameter: (1 byte)
    /// ## Control code (1 byte).
    /// * Control code ”0” means turns off the port;
    /// * Control code ”1” means turns on the port;
    /// # Return Parameter
    /// * Confirmation code: (1 byte)
    ///   - Confirmation code=00H: Port operation complete;
    ///   - Confirmation code=01H: error when receiving package;
    ///   - Confirmation code=1dH: fail to operate the communication port;
    /// # Instruction code
    /// 17H
    /// # Command Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x01
    /// * Package Length          2 byte         0x0004
    /// * Instruction code        1 byte         0x17
    /// * Data                    1 bytes
    ///   - ControlCode           1 byte         0/1
    /// * Checksum                2 bytes        Sum                (see top)
    /// # Acknowledge Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x07
    /// * Package Length          2 byte         0x0003
    /// * Confirmation code       1 byte         xx                 (see above)
    /// * Checksum                2 bytes        Sum                (see top)
    pub async fn Control(&mut self, ctrl: u8) -> Status {
        info!("COMMAND: Control: {=u8:#04x}H", ctrl);

        let mut data: Vec<u8, REPLY_DATA_SIZE> = heapless::Vec::new();
        let _ = data.push(ctrl);

        return self.send_command(Command::Control, data).await;
    }

    /// # Description
    /// Read Module’s status register and system basic configuration parameters.
    /// # Input Parameter
    /// none
    /// # Return Parameter
    /// * Confirmation code: (1 byte) + Basic Parameter List (16 bytes)
    ///   - Confirmation code=00H: read complete;
    ///   - Confirmation code=01H: error when receiving package;
    /// # Instruction code
    /// 0fH
    /// # Command Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x01
    /// * Package Length          2 byte         0x0003
    /// * Instruction code        1 byte         0x0f
    /// * Checksum                2 bytes        Sum                (see top)
    /// # Acknowledge Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x07
    /// * Package Length          2 byte         3+16
    /// * Confirmation code       1 byte         xx                 (see above)
    /// * Data                   16 bytes
    ///   - Basic Param List     16 byte                            (see below)
    /// * Checksum                2 bytes        Sum                (see top)
    /// # System parameters
    /// | Name                   | Description                        | Offset (word) | Size (word) | Size (byte) |
    /// | :---                   | :---                               | :---:         | :---:       | :---:       |
    /// | Status register        | Contents of system status register |     0         |       1     |      2      |
    /// | System identifier code | Fixed value: 0x0009                |     1         |       1     |      2      |
    /// | Finger library size    | Finger library size                |     2         |       1     |      2      |
    /// | Security level         | Security level (1, 2, 3, 4, 5)     |     3         |       1     |      2      |
    /// | Device address         | 32-bit device address              |     4         |       2     |      4      |
    /// | Data packet size       | Size code (0, 1, 2, 3)             |     6         |       1     |      2      |
    /// | Baud settings          | N (baud = 9600*N bps)              |     7         |       1     |      2      |
    ///
    /// | Bit Num     |   15 4   |     3      |  2  |  1   |  0   |
    /// | :---        |   :--:   |    :-:     | :-: | :-:  | :-:  |
    /// | Description | Reserved | ImgBufStat | PWD | Pass | Busy |
    ///
    /// * Busy: 1 bit
    ///   - 1: System is executing commands
    ///   - 0: System is free
    /// * Pass: 1 bit
    ///   - 1: Find the matching finger
    ///   - 0: Wrong finger
    /// * PWD: 1 bit
    ///   - 1: Verified device’s handshaking password
    /// * ImgBufStat: 1 bit
    ///   - 1: Image buffer contains valid image.
    ///
    /// **TODO**: For some reason the device is only sending 32 bytes, even though it
    ///           say 49 in the package! So need to figure out how to get this one
    ///           working. Most of the code is there, the call to the function is just
    ///           disabled. And if someone tries to use it anyway... Well, best of luck :).
    pub async fn ReadSysPara(&mut self) -> Status {
        info!("COMMAND: Read status register and basic configuration parameters");

        let data: Vec<u8, REPLY_DATA_SIZE> = heapless::Vec::new();

        return self.send_command(Command::ReadSysPara, data).await;
    }

    /// # Description
    /// Read the current valid template number of the Module.
    /// # Input Parameter
    /// none
    /// # Return Parameter
    /// * Confirmation code: (1 byte) + Template number (2 bytes)
    ///   - Confirmation code=0x00: read success;
    ///   - Confirmation code=0x01: error when receiving package;
    /// # Instruction code
    /// 1dH
    /// # Command Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x01
    /// * Package Length          2 byte         0x0003
    /// * Instruction code        1 byte         0x1d
    /// * Checksum                2 bytes        0x0021
    /// # Acknowledge Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x07
    /// * Package Length          2 byte         0x0005
    /// * Confirmation code       1 byte         xx                 (see above)
    /// * Data                    2 bytes
    ///   - Template Number       2 byte
    /// * Checksum                2 bytes        Sum                (see top)
    pub async fn TempleteNum(&mut self) -> Status {
        info!("COMMAND: Read current valid template number");

        let data: Vec<u8, REPLY_DATA_SIZE> = heapless::Vec::new();

        return self.send_command(Command::TempleteNum, data).await;
    }

    /// # Description
    /// Read the fingerprint template index table of the module, read the index
    /// table of the fingerprint template up to 256 at a time (32 bytes).
    /// # Input Parameter: (1 byte)
    /// * Index page (1 byte)
    ///   - Index tables are read per page, 256 templates per page
    ///   - Index page 0 means to read 0 ~ 255 fingerprint template index table;
    ///   - Index page 1 means to read 256 ~ 511 fingerprint template index table;
    ///   - Index page 2 means to read 512 ~ 767 fingerprint template index table;
    ///   - Index page 3 means to read 768 ~ 1023 fingerprint template index table
    /// # Return Parameter
    /// * Confirmation code: (1 byte) + Fingerprint Template Index Table (32 bytes)
    ///   - Confirmation code=0x00: read complete;
    ///   - Confirmation code=0x01: error when receiving package;
    /// # Instruction code
    /// 1fH
    /// # Command Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x01
    /// * Package Length          2 byte         0x0004
    /// * Instruction code        1 byte         0x1f
    /// * Data                    1 bytes
    ///   - Index Page            1 byte
    /// * Checksum                2 bytes        Sum                (see top)
    /// # Acknowledge Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x07
    /// * Package Length          2 byte         0x0023
    /// * Confirmation code       1 byte         xx                 (see above)
    /// * Data                   32 bytes
    ///   - Index Page           32 bytes                           (see documentation)
    /// * Checksum                2 bytes        Sum                (see top)
    ///
    /// **TODO**: Return `Status` and ... (32 bytes - `[u8, 32]`)??
    pub async fn ReadIndexTable(&mut self, page: u8) -> Status {
        info!(
            "COMMAND: Read fingerprint template index table: {=u8:#04x}H",
            page
        );

        let mut data: Vec<u8, REPLY_DATA_SIZE> = heapless::Vec::new();
        let _ = data.push(page);

        return self.send_command(Command::ReadIndexTable, data).await;
    }

    // ===== Fingerprint-processing instructions

    /// # Description
    /// Detecting finger and store the detected finger image in ImageBuffer while returning
    /// successfull confirmation code; If there is no finger, returned confirmation code
    /// would be “can’t detect finger”.
    /// # Input Parameter
    /// none
    /// # Return Parameter
    /// * Confirmation code: (1 byte)
    ///   - Confirmation code=00H: finger collection successs;
    ///   - Confirmation code=01H: error when receiving package;
    ///   - Confirmation code=02H: can’t detect finger;
    ///   - Confirmation code=03H: fail to collect finger;
    /// # Instruction code
    /// 01H
    /// # Command Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x01
    /// * Package Length          2 byte         0x0003
    /// * Instruction code        1 byte         0x01
    /// * Checksum                2 bytes        0x05
    /// # Acknowledge Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x07
    /// * Package Length          2 byte         0x0003
    /// * Confirmation code       1 byte         xx                 (see above)
    /// * Checksum                2 bytes        Sum                (see top)
    pub async fn GenImg(&mut self) -> Status {
        info!("COMMAND: Scanning finger");

        let data: Vec<u8, REPLY_DATA_SIZE> = heapless::Vec::new();

        return self.send_command(Command::GenImg, data).await;
    }

    /// # Description
    /// Upload the image in Img_Buffer to upper computer.
    /// # Input Parameter
    /// none
    /// # Return Parameter
    /// * Confirmation code: (1 byte)
    ///   - Confirmation code=00H: ready to transfer the following data packet;
    ///   - Confirmation code=01H: error when receiving package;
    ///   - Confirmation code=0fH: fail to transfer the following data packet;
    /// # Instruction code
    /// 0aH
    /// # Command Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x01
    /// * Package Length          2 byte         0x0003
    /// * Instruction code        1 byte         0x0a
    /// * Checksum                2 bytes        0x000e
    /// # Acknowledge Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x07
    /// * Package Length          2 byte         0x0003
    /// * Confirmation code       1 byte         xx                 (see above)
    /// * Checksum                2 bytes        Sum                (see top)
    pub async fn UpImage(&mut self) -> Status {
        info!("COMMAND: Upload image from image buffer to upper computer");

        let data: Vec<u8, REPLY_DATA_SIZE> = heapless::Vec::new();

        return self.send_command(Command::UpImage, data).await;
    }

    /// # Description
    /// Download image from upper computer to Img_Buffer.
    /// # Input Parameter
    /// none
    /// # Return Parameter
    /// * Confirmation code: (1 byte)
    ///   - Confirmation code=00H: ready to transfer the following data packet;
    ///   - Confirmation code=01H: error when receiving package;
    ///   - Confirmation code=0eH: fail to transfer the following data packet;
    /// # Instruction code
    /// 0bH
    /// # Command Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x01
    /// * Package Length          2 byte         0x0003
    /// * Instruction code        1 byte         0x0b
    /// * Checksum                2 bytes        0x000f
    /// # Acknowledge Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x07
    /// * Package Length          2 byte         0x0003
    /// * Confirmation code       1 byte         xx                 (see above)
    /// * Checksum                2 bytes        Sum                (see top)
    pub async fn DownImage(&mut self) -> Status {
        info!("COMMAND: Download image from upper computer to image buffer");

        let data: Vec<u8, REPLY_DATA_SIZE> = heapless::Vec::new();

        return self.send_command(Command::DownImage, data).await;
    }

    /// # Description
    /// Generate character file from the original finger image in ImageBuffer and store the
    /// file in CharBuffer1 or CharBuffer2.<br>
    /// **Note**: BufferID of CharBuffer1 and CharBuffer2 are 1h and 2h respectively. Other values
    ///           (except 1h, 2h) would be processed as CharBuffer2.
    /// # Input Parameter: (1 byte)
    /// * BufferID - character file buffer number (1 byte).
    /// # Return Parameter
    /// * Confirmation code: (1 byte)
    ///   - Confirmation code=00H: generate character file complete;
    ///   - Confirmation code=01H: error when receiving package;
    ///   - Confirmation code=06H: fail to generate character file due to the over-disorderly fingerprint image;
    ///   - Confirmation code=07H: fail to generate character file due to lackness of character point or
    ///                          over-smallness of fingerprint image;
    ///   - Confirmation code=15H: fail to generate the image for the lackness of valid primary image;
    /// # Instruction code:
    /// 02H
    /// # Command Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x01
    /// * Package Length          2 byte         0x0004
    /// * Instruction code        1 byte         0x02
    /// * Data                    1 bytes
    ///   - BufferID              1 byte         1|2
    /// * Checksum                2 bytes        Sum                (see top)
    /// # Acknowledge Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x07
    /// * Package Length          2 byte         0x0003
    /// * Confirmation code       1 byte         xx                 (see above)
    /// * Checksum                2 bytes        Sum                (see top)
    pub async fn Img2Tz(&mut self, buff: u8) -> Status {
        info!(
            "COMMAND: Generating character file from finger image: {=u8:#04x}H",
            buff
        );

        let mut data: Vec<u8, REPLY_DATA_SIZE> = heapless::Vec::new();
        let _ = data.push(buff);

        return self.send_command(Command::Img2Tz, data).await;
    }

    /// # Description
    /// Combine information of character files from CharBuffer1 and CharBuffer2 and generate
    /// a template which is stored back in both CharBuffer1 and CharBuffer2.
    /// # Input Parameter
    /// none
    /// # Return Parameter
    /// * Confirmation code: (1 byte)
    ///   - Confirmation code=00H: operation success;
    ///   - Confirmation code=01H: error when receiving package;
    ///   - Confirmation code=0aH: fail to combine the character files. That’s, the character
    ///                            files don’t belong to one finger.
    /// # Instruction code
    /// 05H
    /// # Command Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x01
    /// * Package Length          2 byte         0x0003
    /// * Instruction code        1 byte         0x05
    /// * Checksum                2 bytes        0x09
    /// # Acknowledge Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x07
    /// * Package Length          2 byte         0x0003
    /// * Confirmation code       1 byte         xx                 (see above)
    /// * Checksum                2 bytes        Sum                (see top)
    pub async fn RegModel(&mut self) -> Status {
        info!("COMMAND: Generate fingerprint template");

        let data: Vec<u8, REPLY_DATA_SIZE> = heapless::Vec::new();

        return self.send_command(Command::RegModel, data).await;
    }

    /// # Description
    /// Upload the character file or template of CharBuffer1/CharBuffer2 to upper computer.<br>
    /// **Note**: BufferID of CharBuffer1 and CharBuffer2 are 1h and 2h respectively. Other values
    ///           (except 1h, 2h) would be processed as CharBuffer2.
    /// # Input Parameter: (1 byte)
    /// * BufferID - buffer number (1 byte).
    /// # Return Parameter
    /// * Confirmation code: (1 byte)
    ///   - Confirmation code=00H: ready to transfer the following data packet;
    ///   - Confirmation code=01H: error when receiving package;
    ///   - Confirmation code=0dH: error when uploading template;
    /// # Instruction code
    /// 08H
    /// # Command Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x01
    /// * Package Length          2 byte         0x0004
    /// * Instruction code        1 byte         0x08
    /// * Data                    1 bytes
    ///   - BufferID              1 byte
    /// * Checksum                2 bytes        Sum                (see top)
    /// # Acknowledge Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x07
    /// * Package Length          2 byte         0x0003
    /// * Confirmation code       1 byte         xx                 (see above)
    /// * Checksum                2 bytes        Sum                (see top)
    pub async fn UpChar(&mut self, buff: u8) -> Status {
        info!(
            "COMMAND: Upload character file to upper computer: {=u8:#04x}H",
            buff
        );

        let mut data: Vec<u8, REPLY_DATA_SIZE> = heapless::Vec::new();
        let _ = data.push(buff);

        return self.send_command(Command::UpChar, data).await;
    }

    /// # Description
    /// Upper computer download template to module buffer.
    /// # Input Parameter: (1 byte)
    /// * CharBufferID - buffer number (1 byte).
    /// # Return Parameter
    /// * Confirmation code: (1 byte)
    ///   - Confirmation code=00H: ready to transfer the following data packet;
    ///   - Confirmation code=01H: error when receiving package;
    ///   - Confirmation code=0eH: can not receive the following data packet;
    /// # Instruction code
    /// 09H
    /// # Command Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x01
    /// * Package Length          2 byte         0x0004
    /// * Instruction code        1 byte         0x09
    /// * Data                    1 bytes
    ///   - CharBufferID          1 byte
    /// * Checksum                2 bytes        Sum                (see top)
    /// # Acknowledge Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x07
    /// * Package Length          2 byte         0x0003
    /// * Confirmation code       1 byte         xx                 (see above)
    /// * Checksum                2 bytes        Sum                (see top)
    pub async fn DownChar(&mut self, buff: u8) -> Status {
        info!(
            "COMMAND: Download template to model buffer: {=u8:#04x}H",
            buff
        );

        let mut data: Vec<u8, REPLY_DATA_SIZE> = heapless::Vec::new();
        let _ = data.push(buff);

        return self.send_command(Command::DownChar, data).await;
    }

    /// # Description
    /// Store the template of specified buffer (Buffer1/Buffer2) at the designated location
    /// of Flash library.<br>
    /// **Note**: BufferID of CharBuffer1 and CharBuffer2 are 1h and 2h respectively. Other values
    ///           (except 1h, 2h) would be processed as CharBuffer2.
    /// # Input Parameter (3 bytes)
    /// * BufferID - buffer number (1 byte);
    /// * PageID - flash location of the template, two bytes with high byte front and low byte behind (2 bytes)
    /// # Return Parameter
    /// * Confirmation code: (1 byte)
    ///   - Confirmation code=00H: storage success;
    ///   - Confirmation code=01H: error when receiving package;
    ///   - Confirmation code=0bH: addressing PageID is beyond the finger library;
    ///   - Confirmation code=18H: error when writing Flash;
    /// # Instruction code
    /// 06H
    /// # Command Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x01
    /// * Package Length          2 byte         0x0006
    /// * Instruction code        1 byte         0x06
    /// * Data                    3 bytes
    ///   - BufferID              1 byte
    ///   - PageID                2 bytes
    /// * Checksum                2 bytes        Sum                (see top)
    /// # Acknowledge Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x07
    /// * Package Length          2 byte         0x0003
    /// * Confirmation code       1 byte         xx                 (see above)
    /// * Checksum                2 bytes        Sum                (see top)
    pub async fn Store(&mut self, buff: u8, page: u16) -> Status {
        info!(
            "COMMAND: Store fingerprint template in flash: {=u8:#04x}H/{=u16:#06x}H",
            buff, page
        );

        let mut data: Vec<u8, REPLY_DATA_SIZE> = heapless::Vec::new();
        let _ = data.push(buff);

        let split: [u8; 2] = page.to_be_bytes();
        let _ = data.extend(split.iter().map(|&i| i));

        return self.send_command(Command::Store, data).await;
    }

    /// # Description
    /// Load template at the specified location (PageID) of Flash library to template buffer
    /// CharBuffer1/CharBuffer2
    /// # Input Parameter: (3 bytes).
    /// * BufferID - buffer number (1 byte);
    /// * PageID - flash location of the template, two bytes with high byte front and low byte behind (2 bytes).
    /// # Return Parameter
    /// * Confirmation code: (1 byte)
    ///   - Confirmation code=00H: load success;
    ///   - Confirmation code=01H: error when receiving package;
    ///   - Confirmation code=0cH: error when reading template from library or the readout template is invalid;
    ///   - Confirmation code=0bH: addressing PageID is beyond the finger library;
    /// # Instruction code
    /// 07H
    /// # Command Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x01
    /// * Package Length          2 byte         0x0006
    /// * Instruction code        1 byte         0x07
    /// * Data                    3 bytes
    ///   - BufferID              1 byte
    ///   - PageID                2 bytes
    /// * Checksum                2 bytes        Sum                (see top)
    /// # Acknowledge Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x07
    /// * Package Length          2 byte         0x0003
    /// * Confirmation code       1 byte         xx                 (see above)
    /// * Checksum                2 bytes        Sum                (see top)
    pub async fn LoadChar(&mut self, buff: u8, page: u16) -> Status {
        info!(
            "COMMAND: Load template from flash: {=u8:#04x}H/{=u16:#06x}H",
            buff, page
        );

        let mut data: Vec<u8, REPLY_DATA_SIZE> = heapless::Vec::new();
        let _ = data.push(buff);

        let split: [u8; 2] = page.to_be_bytes();
        let _ = data.extend(split.iter().map(|&i| i));

        return self.send_command(Command::LoadChar, data).await;
    }

    /// # Description
    /// Delete a segment (N) of templates of Flash library started from the specified
    /// location (or PageID);
    /// # Input Parameter: (4 bytes)
    /// * PageID - template number in flash (2 bytes).
    /// * N - number of templates to be deleted (2 bytes).
    /// # Return Parameter
    /// * Confirmation code: (1 byte)
    ///   - Confirmation code=00H: delete success;
    ///   - Confirmation code=01H: error when receiving package;
    ///   - Confirmation code=10H: faile to delete templates;
    /// # Instruction code
    /// 0cH
    /// # Command Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x01
    /// * Package Length          2 byte         0x0007
    /// * Instruction code        1 byte         0x0c
    /// * Data                    4 bytes
    ///   - PageID                2 byte
    ///   - N                     2 bytes
    /// * Checksum                2 bytes        Sum                (see top)
    /// # Acknowledge Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x07
    /// * Package Length          2 byte         0x0003
    /// * Confirmation code       1 byte         xx                 (see above)
    /// * Checksum                2 bytes        Sum                (see top)
    pub async fn DeletChar(&mut self, page: u16, n: u16) -> Status {
        info!(
            "COMMAND: Delete a segment of templates in flash: {=u16:#06x}H/{=u16:#06x}H",
            page, n
        );

        let mut data: Vec<u8, REPLY_DATA_SIZE> = heapless::Vec::new();

        let split_page: [u8; 2] = page.to_be_bytes();
        data.extend(split_page.iter().map(|&i| i));

        let split_n: [u8; 2] = page.to_be_bytes();
        data.extend(split_n.iter().map(|&i| i));

        return self.send_command(Command::DeletChar, data).await;
    }

    /// # Description
    /// Delete all the templates in the Flash library.
    /// # Input Parameter
    /// none
    /// # Return Parameter
    /// * Confirmation code: (1 byte)
    ///   - Confirmation code=00H: empty success;
    ///   - Confirmation code=01H: error when receiving package;
    ///   - Confirmation code=11H: fail to clear finger library;
    /// # Instruction code
    /// 0dH
    /// # Command Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x01
    /// * Package Length          2 byte         0x0003
    /// * Instruction code        1 byte         0x0d
    /// * Checksum                2 bytes        0x0011
    /// # Acknowledge Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x07
    /// * Package Length          2 byte         0x0003
    /// * Confirmation code       1 byte         xx                 (see above)
    /// * Checksum                2 bytes        Sum                (see top)
    pub async fn Empty(&mut self) -> Status {
        info!("COMMAND: Delete all templates in flash");

        let data: Vec<u8, REPLY_DATA_SIZE> = heapless::Vec::new();

        return self.send_command(Command::Empty, data).await;
    }

    /// # Description
    /// Carry out precise matching of templates from CharBuffer1 and CharBuffer2,
    /// providing matching results.
    /// # Input Parameter
    /// none
    /// # Return Parameter
    /// * Confirmation code: (1 byte)，matching score.
    ///   - Confirmation code=00H: templates of the two buffers are matching;
    ///   - Confirmation code=01H: error when receiving package;
    ///   - Confirmation code=08H: templates of the two buffers aren’t matching;
    /// # Instruction code
    /// 03H
    /// # Command Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x01
    /// * Package Length          2 byte         0x0003
    /// * Instruction code        1 byte         0x03
    /// * Checksum                2 bytes        0x07
    /// # Acknowledge Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x07
    /// * Package Length          2 byte         0x0005
    /// * Confirmation code       1 byte         xx                 (see above)
    /// * Data                    2 bytes
    ///   - Matching Score        2 byte
    /// * Checksum                2 bytes        Sum                (see top)
    pub async fn Match(&mut self) -> Status {
        info!("COMMAND: Match template");

        let data: Vec<u8, REPLY_DATA_SIZE> = heapless::Vec::new();

        return self.send_command(Command::Match, data).await;
    }

    /// # Description
    /// Search the whole finger library for the template that matches the one in
    /// CharBuffer1 or CharBuffer2. When found, PageID will be returned.
    /// # Input Parameter: (5 bytes).
    /// * BufferID - character file buffer number (1 byte).
    /// * StartPage - searching start address (2 bytes).
    /// * PageNum - searching numbers (2 bytes)
    /// # Return Parameter
    /// * Confirmation code: (1 byte).
    ///   - Confirmation code=00H: found the matching finer;
    ///   - Confirmation code=01H: error when receiving package;
    ///   - Confirmation code=09H: No matching in the library (both the PageID and matching score are 0);
    /// * PageID - matching templates location (2 bytes).
    /// * MatchScore (2 bytes).
    /// # Instruction code
    /// 04H
    /// # Command Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x01
    /// * Package Length          2 byte         0x0008
    /// * Instruction code        1 byte         0x04
    /// * Data                    5 bytes
    ///   - BufferID              1 byte
    ///   - StartPage             2 bytes
    ///   - PageNum               2 bytes
    /// * Checksum                2 bytes        Sum                (see top)
    /// # Acknowledge Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x07
    /// * Package Length          2 byte         0x0007
    /// * Confirmation code       1 byte         xx                 (see above)
    /// * Data                    4 bytes
    ///   - PageID                2 bytes
    ///   - MatchScore            2 bytes
    /// * Checksum                2 bytes        Sum                (see top)
    pub async fn Search(&mut self, buff: u8, start: u16, page: u16) -> Status {
        info!("COMMAND: Search fingerpringt library for template: {=u8:#04x}H/{=u16:#06x}H/{=u16:#06x}H",
        buff, start, page);

        let mut data: Vec<u8, REPLY_DATA_SIZE> = heapless::Vec::new();
        let _ = data.push(buff);

        let split_start: [u8; 2] = start.to_be_bytes();
        let _ = data.extend(split_start.iter().map(|&i| i));

        let split_page: [u8; 2] = page.to_be_bytes();
        let _ = data.extend(split_page.iter().map(|&i| i));

        let res = self.send_command(Command::Search, data).await;

        // Now parse the result.
        if res == Status::CmdExecComplete {
            let page_id = u16::from_be_bytes(self.buffer[10..12].try_into().unwrap());
            let match_score = u16::from_be_bytes(self.buffer[12..14].try_into().unwrap());
            debug!("Search results: PageID={}; MatchScore={}", page_id, match_score);
        }

        return res;
    }

    /// # Description
    /// Detect the finger, record the fingerprint image and store it in ImageBuffer,
    /// return it and record the successful confirmation code;<br>
    /// If no finger is detected, return no finger confirmation code(the module responds
    /// quickly to each instruction, therefore, for continuous detection, cycle
    /// processing is required, which can be limited to the number of cycles or the
    /// total time).<br>
    /// Differences between `GetImageEx` and `GetImage`:<br>
    /// * GetImage: Return the confirmation code 0x00 when the image quality is too bad
    ///             (image collection succeeded).
    /// * GetImageEx: Return the confirmation code 0x07 when the image quality is too
    ///               bad (poor collection quality).
    /// # Input Parameter
    /// none
    /// # Return Parameter
    /// * Confirmation code: (1 byte).
    ///   - Confirmation code=0x00: read success
    ///   - Confirmation code=0x01: error when receiving package;
    ///   - Confirmation code=0x02: no fingers on the sensor;
    ///   - Confirmation code=0x03: unsuccessful entry;
    ///   - Confirmation code=0x07: poor image quality;
    /// # Instruction code
    /// 28H
    /// # Command Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x01
    /// * Package Length          2 byte         0x0003
    /// * Instruction code        1 byte         0x28
    /// * Checksum                2 bytes        Sum                (see top)
    /// # Acknowledge Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x07
    /// * Package Length          2 byte         0x0003
    /// * Confirmation code       1 byte         xx                 (see above)
    /// * Checksum                2 bytes        Sum                (see top)
    pub async fn GetImageEx(&mut self) -> Status {
        info!("COMMAND: Scan finger, record image and store it buffer");

        let data: Vec<u8, REPLY_DATA_SIZE> = heapless::Vec::new();

        return self.send_command(Command::GetImageEx, data).await;
    }

    /// # Description
    /// Cancel instruction
    /// # Input Parameter
    /// none
    /// # Return Parameter
    /// * Confirmation code: (1 byte).
    ///   - Confirmation code=0x00: cancel setting successful;
    ///   - Confirmation code=other: cancel setting failed;
    /// # Instruction code
    /// 30H
    /// # Command Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x01
    /// * Package Length          2 byte         0x0003
    /// * Instruction code        1 byte         0x30
    /// * Checksum                2 bytes        Sum                (see top)
    /// # Acknowledge Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x07
    /// * Package Length          2 byte         0x0003
    /// * Confirmation code       1 byte         xx                 (see above)
    /// * Checksum                2 bytes        Sum                (see top)
    pub async fn Cancel(&mut self) -> Status {
        info!("COMMAND: Cancel instruction");

        let data: Vec<u8, REPLY_DATA_SIZE> = heapless::Vec::new();

        return self.send_command(Command::Cancel, data).await;
    }

    /// # Description
    /// Send handshake instructions to the module. If the module works normally, the
    /// confirmation code 0x00 will be returned. The upper computer can continue to
    /// send instructions to the module.If the confirmation code is other or no reply,
    /// it means that the device is abnormal.
    /// # Input Parameter
    /// none
    /// # Return Parameter
    /// * Confirmation code: (1 byte).
    ///   - Confirmation code=0x00: the device is normal and can receive instructions;
    ///   - Confirmation code=other: the device is abnormal;
    /// # Instruction code
    /// 40H
    /// # Command Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x01
    /// * Package Length          2 byte         0x0003
    /// * Instruction code        1 byte         0x40
    /// * Checksum                2 bytes        Sum                (see top)
    /// # Acknowledge Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x07
    /// * Package Length          2 byte         0x0003
    /// * Confirmation code       1 byte         xx                 (see above)
    /// * Checksum                2 bytes        Sum                (see top)
    pub async fn HandShake(&mut self) -> Status {
        info!("COMMAND: Handshake");

        let data: Vec<u8, REPLY_DATA_SIZE> = heapless::Vec::new();

        return self.send_command(Command::HandShake, data).await;
    }

    /// # Description
    /// Check whether the sensor is normal.
    /// # Input Parameter
    /// none
    /// # Return Parameter
    /// * Confirmation code: (1 byte).
    ///   - Confirmation code=0x00: the sensor is normal;
    ///   - Confirmation code=0x29: the sensor is abnormal;
    /// # Instruction code
    /// 36H
    /// # Command Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x01
    /// * Package Length          2 byte         0x0003
    /// * Instruction code        1 byte         0x36
    /// * Checksum                2 bytes        Sum                (see top)
    /// # Acknowledge Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x07
    /// * Package Length          2 byte         0x0003
    /// * Confirmation code       1 byte         xx                 (see above)
    /// * Checksum                2 bytes        Sum                (see top)
    pub async fn CheckSensor(&mut self) -> Status {
        info!("COMMAND: Checking sensor");

        let data: Vec<u8, REPLY_DATA_SIZE> = heapless::Vec::new();

        return self.send_command(Command::CheckSensor, data).await;
    }

    /// # Description
    /// Get the algorithm library version.
    /// # Input Parameter
    /// none
    /// # Return Parameter
    /// * Confirmation code: (1 byte)
    ///   - Confirmation code=0x00: success;
    ///   - Confirmation code=0x01: error when receiving package;
    /// * AlgVer - algorithm library version string (32 bytes).
    /// # Instruction code
    /// 39H
    /// # Command Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x01
    /// * Package Length          2 byte         0x0003
    /// * Instruction code        1 byte         0x39
    /// * Checksum                2 bytes        Sum                (see top)
    /// # Acknowledge Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x07
    /// * Package Length          2 byte         0x0023
    /// * Confirmation code       1 byte         xx                 (see above)
    /// * Data                   32 bytes
    ///   - AlgVer               32 bytes
    /// * Checksum                2 bytes        Sum                (see top)
    ///
    /// **TODO**: Return `Status` and AlgVer (32 bytes - `[u8, 32]`)??
    pub async fn GetAlgVer(&mut self) -> Status {
        info!("COMMAND: Get algorithm library version");

        let data: Vec<u8, REPLY_DATA_SIZE> = heapless::Vec::new();

        return self.send_command(Command::GetAlgVer, data).await;
    }

    /// # Description
    /// Get the firmware version.
    /// # Input Parameter
    /// none
    /// # Return Parameter
    /// * Confirmation code (1 byte).
    ///   - Confirmation code=0x00: success;
    ///   - Confirmation code=0x01: error when receiving package;
    /// * FwVer - firmware version string (32 bytes).
    /// # Instruction code
    /// 3aH
    /// # Command Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x01
    /// * Package Length          2 byte         0x0003
    /// * Instruction code        1 byte         0x3a
    /// * Checksum                2 bytes        Sum                (see top)
    /// # Acknowledge Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x07
    /// * Package Length          2 byte         0x0023
    /// * Confirmation code       1 byte         xx                 (see above)
    /// * Data                   32 bytes
    ///   - FwVer                32 bytes
    /// * Checksum                2 bytes        Sum                (see top)
    ///
    /// **TODO**: Return `Status` and FwVer (32 bytes - `[u8, 32]`)??
    pub async fn GetFwVer(&mut self) -> Status {
        info!("COMMAND: Get firmware version");

        let data: Vec<u8, REPLY_DATA_SIZE> = heapless::Vec::new();

        return self.send_command(Command::GetFwVer, data).await;
    }

    /// # Description
    /// Read product information.
    /// # Input Parameter
    /// none
    /// # Return Parameter
    /// * Confirmation code (1 byte).
    ///   - Confirmation code=0x00: success;
    ///   - Confirmation code=0x01: error when receiving package;
    /// * ProdInfo - product information (46 bytes).
    /// # Instruction code
    /// 3cH
    /// # Command Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x01
    /// * Package Length          2 byte         0x0003
    /// * Instruction code        1 byte         0x3c
    /// * Checksum                2 bytes        Sum                (see top)
    /// # Acknowledge Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x07
    /// * Package Length          2 byte         0x0031
    /// * Confirmation code       1 byte         xx                 (see above)
    /// * Data                   46 bytes
    ///   - ProdInfo             46 bytes                           (see below)
    /// * Checksum                2 bytes        Sum                (see top)
    /// # Product information
    /// | Code             | Bytes | Meaning |
    /// | :---             | ---:  | :---    |
    /// | PARAM_FPM_MODEL  |   16  | Module type, ASCII |
    /// | PARAM_BN         |    4  | Module batch number, ASCII |
    /// | PARAM_SN         |    8  | Module serial number, ASCII |
    /// | PARAM_HW_VER     |    2  | For the hardware version, the first byte represents the main version and the second byte represents the sub-version |
    /// | PARAM_FPS_MODEL  |    8  | Sensor type, ASCII |
    /// | PARAM_FPS_WIDTH  |    2  | Sensor image width |
    /// | PARAM_FPS_HEIGHT |    2  | Sensor image height |
    /// | PARAM_TMPL_SIZE  |    2  | Template size |
    /// | PARAM_TMPL_TOTAL |    2  | Fingerprint database size |
    pub async fn ReadProdInfo(&mut self) -> Status {
        info!("COMMAND: Read product information");

        let data: Vec<u8, REPLY_DATA_SIZE> = heapless::Vec::new();

        return self.send_command(Command::ReadProdInfo, data).await;
    }

    /// # Description
    /// Send soft reset instruction to the module. If the module works normally,
    /// return confirmation code 0x00, and then perform reset operation.
    /// # Input Parameter
    /// none
    /// # Return Parameter
    /// * Confirmation code: (1 byte)
    ///   - Confirmation code=0x00: success;
    ///   - Confirmation code=other: device is abnormal
    /// # Instruction code
    /// 3dH
    /// # Command Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x01
    /// * Package Length          2 byte         0x0003
    /// * Instruction code        1 byte         0x3d
    /// * Checksum                2 bytes        Sum                (see top)
    /// # Acknowledge Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x07
    /// * Package Length          2 byte         0x0003
    /// * Confirmation code       1 byte         xx                 (see above)
    /// * Checksum                2 bytes        Sum                (see top)
    pub async fn SoftRst(&mut self) -> Status {
        info!("COMMAND: Soft reset");

        let data: Vec<u8, REPLY_DATA_SIZE> = heapless::Vec::new();

        return self.send_command(Command::SoftRst, data).await;
    }

    /// # Description
    /// Aura LED control
    /// # Input Parameter: (4 bytes)
    /// * Control: (1 byte)
    ///   - 0x01: Breathing light
    ///   - 0x02: Flashing light
    ///   - 0x03: Light Always on
    ///   - 0x04: Light Always off
    ///   - 0x05: Light gradually on
    ///   - 0x06: Light gradually off
    /// * Speed: (1 byte)
    ///   - 0x00-0xff, 256 gears, Minimum 5s cycle.
    /// * ColorIndex: (1 byte)
    ///   - 0x01: Red
    ///   - 0x02: Blue
    ///   - 0x03: Purple
    /// * Times: (1 byte)
    ///   - Number of cycles: 0- infinite, 1-255.
    /// # Return Parameter
    /// * Confirmation code (1 byte).
    ///   - Confirmation code=0x00: success;
    ///   - Confirmation code=0x01: error when receiving package;
    /// # Instruction code
    /// 35H
    /// # Command Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x01
    /// * Package Length          2 byte         0x0007
    /// * Instruction code        1 byte         0x35
    /// * Data                    4 bytes
    ///   - Control code          1 byte         Ctrl               (see above)
    ///   - Speed                 1 byte         Speed              (see above)
    ///   - Colour index          1 byte         ColourIndex        (see above)
    ///   - Times                 1 byte         Times              (see above)
    /// * Checksum                2 bytes        Sum                (see top)
    /// # Acknowledge Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x07
    /// * Package Length          2 byte         0x0003
    /// * Confirmation code       1 byte         xx                 (see above)
    /// * Checksum                2 bytes        Sum                (see top)
    pub async fn AuraLedConfig(
        &mut self,
        ctrl: AuroraLEDControl,
        speed: u8,
        colour: AuroraLEDColour,
        times: u8,
    ) -> Status {
        info!(
            "COMMAND: Setting up aura LED: {=u8:#04x}H/{=u8:#04x}H/{=u8:#04x}H/{=u8:#04x}H",
            ctrl as u8, speed, colour as u8, times
        );

        let mut data: Vec<u8, REPLY_DATA_SIZE> = heapless::Vec::new();
        let _ = data.push(ctrl as u8);
        let _ = data.push(speed);
        let _ = data.push(colour as u8);
        let _ = data.push(times);

        return self.send_command(Command::AuraLedConfig, data).await;
    }

    // ===== Other instructions

    /// # Description
    /// Command the Module to generate a random number and return it to upper computer.
    /// # Input Parameter
    /// none
    /// # Return Parameter
    /// * Confirmation code: (1 byte)
    ///   - Confirmation code=00H: generation success;
    ///   - Confirmation code=01H: error when receiving package;
    /// # Instruction code
    /// 14H
    /// # Command Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x01
    /// * Package Length          2 byte         0x0003
    /// * Instruction code        1 byte         0x14
    /// * Checksum                2 bytes        0x0018
    /// # Acknowledge Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x07
    /// * Package Length          2 byte         0x0007
    /// * Confirmation code       1 byte         xx                 (see above)
    /// * Data                    4 bytes
    ///   - Random Number         4 bytes
    /// * Checksum                2 bytes        Sum                (see top)
    pub async fn GetRandomCode(&mut self) -> Status {
        info!("COMMAND: Get random code");

        let data: Vec<u8, REPLY_DATA_SIZE> = heapless::Vec::new();

        return self.send_command(Command::GetRandomCode, data).await;
    }

    /// # Description
    /// Read information page (512bytes).
    /// # Input Parameter
    /// none
    /// # Return Parameter
    /// * Confirmation code: (1 byte)
    ///   - Confirmation code=00H: ready to transfer the following data packet;
    ///   - Confirmation code=01H: error when receiving package;
    ///   - Confirmation code=0fH: can not transfer the following data packet;
    /// # Instruction code
    /// 16H
    /// # Command Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x01
    /// * Package Length          2 byte         0x0003
    /// * Instruction code        1 byte         0x16
    /// * Checksum                2 bytes        Sum                (see top)
    /// # Acknowledge Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x07
    /// * Package Length          2 byte         0x0003
    /// * Confirmation code       1 byte         xx                 (see above)
    /// * Checksum                2 bytes        Sum                (see top)
    pub async fn ReadInfPage(&mut self) -> Status {
        info!("COMMAND: Read information page");

        let data: Vec<u8, REPLY_DATA_SIZE> = heapless::Vec::new();

        return self.send_command(Command::ReadInfPage, data).await;
    }

    /// # Description
    /// Upper computer to write data to the specified Flash page. Also see [`R503::ReadNotepad`].
    /// # Input Parameter
    /// * PageNumber - notepad page number (1 byte).
    /// * Content - data (32 bytes).
    /// # Return Parameter
    /// * Confirmation code: (1 byte)
    ///   - Confirmation code=00H: write success;
    ///   - Confirmation code=01H: error when receiving package;
    /// # Instruction code
    /// 18H
    /// # Command Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x01
    /// * Package Length          2 byte         0x0036
    /// * Instruction code        1 byte         0x18
    /// * Data                   33 bytes
    ///   - PageNumber            1 byte
    ///   - Content              32 bytes
    /// * Checksum                2 bytes        Sum                (see top)
    /// # Acknowledge Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x07
    /// * Package Length          2 byte         0x0003
    /// * Confirmation code       1 byte         xx                 (see above)
    /// * Checksum                2 bytes        Sum                (see top)
    pub async fn WriteNotepad(&mut self, page: u8, content: &[u8; 32]) -> Status {
        info!("COMMAND: Write notepad: {=u8:#04x}H/<content>", page); // Not sure how to output a `&[u128; 2]`.

        let mut data: Vec<u8, REPLY_DATA_SIZE> = heapless::Vec::new();
        let _ = data.push(page);
        let _ = data.extend(content.iter().map(|&i| i));

        return self.send_command(Command::WriteNotepad, data).await;
    }

    /// # Description
    /// Read the specified page’s data content. Also see [`R503::WriteNotepad`].
    /// # Input Parameter
    /// none
    /// # Return Parameter
    /// * Confirmation code: (1 byte)
    ///   - Confirmation code=00H: read success;
    ///   - Confirmation code=01H: error when receiving package;
    /// * Data content: (32 bytes)
    /// # Instruction code
    /// 19H
    /// # Command Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x01
    /// * Package Length          2 byte         0x0004
    /// * Instruction code        1 byte         0x19
    /// * Data                    1 bytes
    ///   - PageNumber            1 byte
    /// * Checksum                2 bytes        Sum                (see top)
    /// # Acknowledge Package format
    /// * Header                  2 bytes        0xEF01
    /// * Address                 4 bytes        xxxxxx
    /// * Package Identifier      1 byte         0x07
    /// * Package Length          2 byte         3+32
    /// * Confirmation code       1 byte         xx                 (see above)
    /// * Data                   32 bytes
    ///   - User Content         32 bytes
    /// * Checksum                2 bytes        Sum                (see top)
    ///
    /// **TODO**: Return `Status` and User Content (32 bytes - `[u8, 32]`)??
    pub async fn ReadNotepad(&mut self) -> Status {
        info!("COMMAND: Read notepad");

        let data: Vec<u8, REPLY_DATA_SIZE> = heapless::Vec::new();

        return self.send_command(Command::ReadNotepad, data).await;
    }

    // ===== Wrapper functions
    // Just to simplify life a little. Should just return `true` (failure) or `false` (success).

    /// # Description
    /// Set the Aura to slow blinking red.
    /// # Input Parameter
    /// none
    /// # Return Parameter
    /// true or false
    pub async fn Wrapper_AuraSet_BlinkinRedSlow(&mut self) -> bool {
        match self
            .AuraLedConfig(
                AuroraLEDControl::BreathingLight,
                AuroraLEDSpeed::Slow as u8,
                AuroraLEDColour::Red,
                0,
            )
            .await
        {
            Status::CmdExecComplete => {
                info!("Fingerprint scanner LED set to blinking red, slow");
                return true;
            }
            Status::ErrorReceivePackage => {
                error!("Package receive: Wrapper_AuraSet_BlinkinRedSlow()");
            }
            stat => {
                error!(
                    "Unknown return code='{=u8:#04x}': Wrapper_AuraSet_BlinkinRedSlow()",
                    stat as u8
                );
            }
        }

        return false;
    }

    /// # Description
    /// Set the Aura to medium blinking red.
    /// # Input Parameter
    /// none
    /// # Return Parameter
    /// true or false
    pub async fn Wrapper_AuraSet_BlinkinRedMedium(&mut self) -> bool {
        match self
            .AuraLedConfig(
                AuroraLEDControl::BreathingLight,
                AuroraLEDSpeed::Medium as u8,
                AuroraLEDColour::Red,
                0,
            )
            .await
        {
            Status::CmdExecComplete => {
                info!("Fingerprint scanner LED set to blinking red, medium");
                return true;
            }
            Status::ErrorReceivePackage => {
                error!("Package receive: Wrapper_AuraSet_BlinkinRedMedium()");
            }
            stat => {
                error!(
                    "Unknown return code='{=u8:#04x}': Wrapper_AuraSet_BlinkinRedMedium()",
                    stat as u8
                );
            }
        }

        return false;
    }

    /// # Description
    /// Set the Aura to fast blinking red.
    /// # Input Parameter
    /// none
    /// # Return Parameter
    /// true or false
    pub async fn Wrapper_AuraSet_BlinkinRedFast(&mut self) -> bool {
        match self
            .AuraLedConfig(
                AuroraLEDControl::BreathingLight,
                AuroraLEDSpeed::Fast as u8,
                AuroraLEDColour::Red,
                0,
            )
            .await
        {
            Status::CmdExecComplete => {
                info!("Fingerprint scanner LED set to blinking red, fast");
                return true;
            }
            Status::ErrorReceivePackage => {
                error!("Package receive: Wrapper_AuraSet_BlinkinRedFast()");
            }
            stat => {
                error!(
                    "Unknown return code='{=u8:#04x}': Wrapper_AuraSet_BlinkinRedFast()",
                    stat as u8
                );
            }
        }

        return false;
    }

    /// # Description
    /// Set the Aura to steady red.
    /// # Input Parameter
    /// none
    /// # Return Parameter
    /// true or false
    pub async fn Wrapper_AuraSet_SteadyRed(&mut self) -> bool {
        match self
            .AuraLedConfig(AuroraLEDControl::AlwaysOn, 0, AuroraLEDColour::Red, 0)
            .await
        {
            Status::CmdExecComplete => {
                info!("Fingerprint scanner LED set to steady red");
                return true;
            }
            Status::ErrorReceivePackage => {
                error!("Package receive: Wrapper_AuraSet_SteadyRed()");
            }
            stat => {
                error!(
                    "Unknown return code='{=u8:#04x}': Wrapper_AuraSet_SteadyRed()",
                    stat as u8
                );
            }
        }

        return false;
    }

    // -----

    /// # Description
    /// Set the Aura to medium blinking blue.
    /// # Input Parameter
    /// none
    /// # Return Parameter
    /// true or false
    pub async fn Wrapper_AuraSet_BlinkinBlueMedium(&mut self) -> bool {
        match self
            .AuraLedConfig(
                AuroraLEDControl::BreathingLight,
                AuroraLEDSpeed::Medium as u8,
                AuroraLEDColour::Blue,
                0,
            )
            .await
        {
            Status::CmdExecComplete => {
                info!("Fingerprint scanner LED set to blinking blue, medium");
                return true;
            }
            Status::ErrorReceivePackage => {
                error!("Package receive: Wrapper_AuraSet_BlinkinBlueMedium()");
            }
            stat => {
                error!(
                    "Unknown return code='{=u8:#04x}': Wrapper_AuraSet_BlinkinBlueMedium()",
                    stat as u8
                );
            }
        }

        return false;
    }

    /// # Description
    /// Set the Aura to steady blue.
    /// # Input Parameter
    /// none
    /// # Return Parameter
    /// true or false
    pub async fn Wrapper_AuraSet_SteadyBlue(&mut self) -> bool {
        match self
            .AuraLedConfig(AuroraLEDControl::AlwaysOn, 0, AuroraLEDColour::Blue, 0)
            .await
        {
            Status::CmdExecComplete => {
                info!("Fingerprint scanner LED set to steady blue");
                return true;
            }
            Status::ErrorReceivePackage => {
                error!("Package receive: Wrapper_AuraSet_SteadyBlue()");
            }
            stat => {
                error!(
                    "Unknown return code='{=u8:#04x}': Wrapper_AuraSet_SteadyBlue()",
                    stat as u8
                );
            }
        }

        return false;
    }

    /// # Description
    /// Set the Aura to medium blinking purple.
    /// # Input Parameter
    /// none
    /// # Return Parameter
    /// true or false
    pub async fn Wrapper_AuraSet_BlinkinPurpleMedium(&mut self) -> bool {
        match self
            .AuraLedConfig(
                AuroraLEDControl::BreathingLight,
                AuroraLEDSpeed::Medium as u8,
                AuroraLEDColour::Purple,
                0,
            )
            .await
        {
            Status::CmdExecComplete => {
                info!("Fingerprint scanner LED set to blinking purple, medium");
                return true;
            }
            Status::ErrorReceivePackage => {
                error!("Package receive: Wrapper_AuraSet_BlinkinPurpleMedium()");
            }
            stat => {
                error!(
                    "Unknown return code='{=u8:#04x}': Wrapper_AuraSet_BlinkinPurpleMedium()",
                    stat as u8
                );
            }
        }

        return false;
    }

    /// # Description
    /// Set the Aura to steady purple.
    /// # Input Parameter
    /// none
    /// # Return Parameter
    /// true or false
    pub async fn Wrapper_AuraSet_SteadyPurpe(&mut self) -> bool {
        match self
            .AuraLedConfig(AuroraLEDControl::AlwaysOn, 0, AuroraLEDColour::Purple, 0)
            .await
        {
            Status::CmdExecComplete => {
                info!("Fingerprint scanner LED set to steady blue");
                return true;
            }
            Status::ErrorReceivePackage => {
                error!("Package receive: Wrapper_AuraSet_SteadyPurpe()");
            }
            stat => {
                error!(
                    "Unknown return code='{=u8:#04x}': Wrapper_AuraSet_SteadyPurpe()",
                    stat as u8
                );
            }
        }

        return false;
    }

    // -----

    /// # Description
    /// Turn off the Aura.
    /// # Input Parameter
    /// none
    /// # Return Parameter
    /// true or false
    pub async fn Wrapper_AuraSet_Off(&mut self) -> bool {
        match self
            .AuraLedConfig(AuroraLEDControl::AlwaysOff, 0, AuroraLEDColour::Purple, 0)
            .await
        {
            Status::CmdExecComplete => {
                info!("Fingerprint scanner LED turned off");
                return true;
            }
            Status::ErrorReceivePackage => {
                error!("Package receive: Wrapper_AuraSet_Off()");
            }
            stat => {
                error!(
                    "Unknown return code='{=u8:#04x}': Wrapper_AuraSet_Off()",
                    stat as u8
                );
            }
        }

        return false;
    }

    /// # Description
    /// Setup the fingerprint scanner.
    /// # Input Parameter
    /// none
    /// # Return Parameter
    /// true or false
    /// # Runs:
    /// * [`R503::VfyPwd`]
    /// * [`R503::CheckSensor`]
    /// * [`R503::ReadSysPara`]
    pub async fn Wrapper_Setup(&mut self) -> bool {
        match self.VfyPwd(self.password).await {
            Status::CmdExecComplete => {
                info!("Fingerprint scanner password matches");
                // Fall through..
            }
            Status::ErrorReceivePackage => {
                error!("Package receive: Wrapper_Setup()/VfyPwd()");

                self.Wrapper_AuraSet_BlinkinRedMedium().await;
                return false;
            }
            Status::ErrorPassword => {
                error!("Wrong password");

                self.Wrapper_AuraSet_BlinkinRedFast().await;
                return false;
            }
            stat => {
                error!(
                    "Unknown return code='{=u8:#04x}': Wrapper_Setup()/VfyPwd()",
                    stat as u8
                );

                self.Wrapper_AuraSet_Off().await;
                return false;
            }
        }

        match self.SoftRst().await {
            Status::CmdExecComplete => {
                info!("Reset successful");
                // Fall through..
            }
            Status::ErrorReceivePackage => {
                error!("Package receive: Wrapper_Setup()/SoftRst()");

                self.Wrapper_AuraSet_BlinkinRedMedium().await;
                return false;
            }
            stat => {
                error!(
                    "Unknown return code='{=u8:#04x}': Wrapper_Setup()/SoftRst()",
                    stat as u8
                );

                self.Wrapper_AuraSet_Off().await;
                return false;
            }
        }
        Timer::after_millis(500).await; // Give it half a second to come back up.

        match self.CheckSensor().await {
            Status::CmdExecComplete => {
                info!("Sensor is normal");
                // Fall through..
            }
            Status::ErrorSensorAbnormal => {
                error!("Sensor is abnormal: Wrapper_Setup()/CheckSensor()");
                return false;
            }
            Status::ErrorReceivePackage => {
                error!("Package receive: Wrapper_Setup()/CheckSensor()");

                self.Wrapper_AuraSet_BlinkinRedMedium().await;
                return false;
            }
            stat => {
                error!(
                    "Unknown return code='{=u8:#04x}': Wrapper_Setup()/CheckSensor()",
                    stat as u8
                );

                self.Wrapper_AuraSet_Off().await;
                return false;
            }
        }

        match self.ReadSysPara().await {
            Status::CmdExecComplete => {
                info!("System parameters read");
                // Fall through..
            }
            Status::ErrorReceivePackage => {
                error!("Package receive: Wrapper_Setup()/ReadSysPara()");

                self.Wrapper_AuraSet_BlinkinRedMedium().await;
                return false;
            }
            stat => {
                error!(
                    "Unknown return code='{=u8:#04x}': Wrapper_Setup()/ReadSysPara()",
                    stat as u8
                );

                self.Wrapper_AuraSet_Off().await;
                return false;
            }
        }

        // TODO: The device is only sending 32 bytes, even though it
        //       say in the package that the package length is 49!
        // match self.ReadProdInfo().await {
        //     Status::CmdExecComplete => {
        //         info!("Product information read");
        //         // Fall through..
        //     }
        //     Status::ErrorReceivePackage => {
        //         error!("Package receive: Wrapper_Setup()/ReadProdInfo()");

        //         self.Wrapper_AuraSet_BlinkinRedMedium().await;
        //         return false;
        //     }
        //     stat => {
        //         error!(
        //             "Unknown return code='{=u8:#04x}': Wrapper_Setup()/ReadProdInfo()",
        //             stat as u8
        //         );

        //         self.Wrapper_AuraSet_Off().await;
        //         return false;
        //     }
        // }

        match self.TempleteNum().await {
            Status::CmdExecComplete => {
                info!("Template number read");
                // Fall through..
            }
            Status::ErrorReceivePackage => {
                error!("Package receive: Wrapper_Setup()/TempleteNum()");

                self.Wrapper_AuraSet_BlinkinRedMedium().await;
                return false;
            }
            stat => {
                error!(
                    "Unknown return code='{=u8:#04x}': Wrapper_Setup()/TempleteNum()",
                    stat as u8
                );

                self.Wrapper_AuraSet_Off().await;
                return false;
            }
        }

        return true;
    }

    /// # Description
    /// Get fingerprint, turn the Aura to appropriate colour on fail/success.
    /// # Input Parameter
    /// Storage buffer (one of two)
    /// # Return Parameter
    /// true or false
    /// # Runs:
    /// * [`R503::GenImg`]
    /// * [`R503::Img2Tz`]
    pub async fn Wrapper_Get_Fingerprint(&mut self, store: u8) -> bool {
        // =====
        // Sometimes this scanner is a bit .. "sensitive". If you don't place your finger EXACTLY
        // right, it returns "no finger on sensor" (`ErrorNoFingerOnSensor`)!
        self.Wrapper_AuraSet_BlinkinBlueMedium().await;

        info!("Place the finger on the scanner");
        if self.wakeup.get_level() == Level::High {
            self.wakeup.wait_for_low().await;
        } else {
            self.wakeup.wait_for_high().await;
        }
        debug!("  Finger detected");

        // Scan the finger.
        info!("Generating image");
        match self.GenImg().await {
            Status::CmdExecComplete => {
                info!("Successfully got image");

                self.Wrapper_AuraSet_SteadyBlue().await;
                // Fall through..
            }
            Status::ErrorReceivePackage => {
                error!("Package receive: Wrapper_Get_Fingerprint()/GenImg()");

                self.Wrapper_AuraSet_BlinkinRedMedium().await;
                return false;
            }
            Status::ErrorNoFingerOnSensor => {
                error!("No finger on sensor: Wrapper_Get_Fingerprint()/GenImg()");

                self.Wrapper_AuraSet_BlinkinRedSlow().await;
                return false;
            }
            Status::ErrorEnroleFinger => {
                error!("Failed to enrole finger: Wrapper_Get_Fingerprint()/GenImg()");

                self.Wrapper_AuraSet_BlinkinRedMedium().await;
                return false;
            }
            stat => {
                error!(
                    "Unknown return code='{=u8:#04x}': Wrapper_Get_Fingerprint()/GenImg()",
                    stat as u8
                );

                self.Wrapper_AuraSet_Off().await;
                return false;
            }
        }

        // Generate character file from the finger image.
        debug!("Generate character file");
        match self.Img2Tz(store).await {
            Status::CmdExecComplete => {
                info!("Successfully generated character from fingerprint image");
                return true;
            }
            Status::ErrorReceivePackage => {
                error!("Package receive: Wrapper_Get_Fingerprint()/Img2Tz()");

                self.Wrapper_AuraSet_BlinkinRedMedium().await;
                return false;
            }
            Status::ErrorGenCharFileDistortedImage => {
                error!("Failed to generate character file due to distorted fingerprint image");

                self.Wrapper_AuraSet_BlinkinRedMedium().await;
                return false;
            }
            Status::ErrorGenCharFileSmallImage => {
                error!("Failed to generate character file due to too small image");

                self.Wrapper_AuraSet_BlinkinRedMedium().await;
                return false;
            }
            Status::ErrorMissingValidPrimaryImage => {
                error!("Failed to generate image because of lac of valid primary image");

                self.Wrapper_AuraSet_BlinkinRedMedium().await;
                return false;
            }
            stat => {
                error!(
                    "Unknown return code='{=u8:#04x}': Wrapper_Get_Fingerprint()/Img2Tz()",
                    stat as u8
                );

                self.Wrapper_AuraSet_Off().await;
                return false;
            }
        }
    }

    /// # Description
    /// Enrole fingerprint into memory, turn the Aura to appropriate colour on fail/success.
    /// # Input Parameter
    /// Buffer storage (1-200)
    /// # Return Parameter
    /// true or false
    /// # Runs:
    /// * [`R503::Wrapper_Setup`]
    /// * [`R503::Wrapper_Get_Fingerprint`] - twice, one for each buffer.
    /// * [`R503::RegModel`]
    /// * [`R503::Store`]
    pub async fn Wrapper_Enrole_Fingerprint(&mut self, buffer: u16) -> bool {
        // NOTE: In case of error here, don't set the aura, it's done in the sub functions!

        // =====
        // 1) Verify the password.
        if !self.Wrapper_Setup().await {
            error!("Can't setup scanner");
            return false;
        } else {
            info!("Setup complete");

            if !self.Wrapper_AuraSet_SteadyBlue().await {
                error!("Can't set colour steady blue");
                return false;
            } else {
                // =====
                // 2) Get the fingerprint - #1.
                if !self.Wrapper_Get_Fingerprint(1).await {
                    error!("Couldn't scan the finger");
                    return false;
                } else {
                    info!("Scanned and saved the finger (#1)");

                    self.Wrapper_AuraSet_Off().await;
                    Timer::after_secs(1).await;

                    // =====
                    // 3) Get the fingerprint - #2.
                    if !self.Wrapper_Get_Fingerprint(2).await {
                        error!("Couldn't scan the finger (second time)");
                        return false;
                    } else {
                        info!("Scanned and saved the finger (#2)");

                        // =====
                        // 4) Create a fingerprint model.
                        match self.RegModel().await {
                            Status::CmdExecComplete => {
                                info!("Fingerprint model generated");
                                // Fall through..
                            }
                            Status::ErrorReceivePackage => {
                                error!("Package receive: Wrapper_Enrole_Fingerprint()/RegModel()");

                                self.Wrapper_AuraSet_BlinkinRedMedium().await;
                                return false;
                            }
                            Status::ErrorCombineCharFiles => {
                                error!("Failed to combine character files");

                                self.Wrapper_AuraSet_BlinkinRedMedium().await;
                                return false;
                            }
                            stat => {
                                error!("Unknown return code='{=u8:#04x}': Wrapper_Enrole_Fingerprint()/RegModel()", stat as u8);

                                self.Wrapper_AuraSet_Off().await;
                                return false;
                            }
                        }

                        // =====
                        // 5) Store the fingerprint model in the flash.
                        match self.Store(0x01, buffer).await {
                            Status::CmdExecComplete => {
                                info!(
                                    "Fingerprint model stored in the flash (buffer={:?})",
                                    buffer
                                );

                                self.Wrapper_AuraSet_Off().await;
                                return true;
                            }
                            Status::ErrorReceivePackage => {
                                error!("Package receive: Wrapper_Enrole_Fingerprint()/Store()");

                                self.Wrapper_AuraSet_BlinkinRedMedium().await;
                                return false;
                            }
                            Status::ErrorPageIdBeyondLibrary => {
                                error!("Package ID beyond library");

                                self.Wrapper_AuraSet_BlinkinRedMedium().await;
                                return false;
                            }
                            Status::ErrorWriteFlash => {
                                error!("Can't write flash");

                                self.Wrapper_AuraSet_BlinkinRedFast().await;
                                return false;
                            }
                            stat => {
                                error!("Unknown return code='{=u8:#04x}': Wrapper_Enrole_Fingerprint()/Store()", stat as u8);

                                self.Wrapper_AuraSet_Off().await;
                                return false;
                            }
                        }
                    }
                }
            }
        }
    }

    /// # Description
    /// Verify fingerprint, turn the Aura to appropriate colour on fail/success.
    /// # Input Parameter
    /// none
    /// # Return Parameter
    /// true or false
    /// # Runs:
    /// * [`R503::Wrapper_Setup`]
    /// * [`R503::Wrapper_Get_Fingerprint`] - only once, we only need one buffer.
    /// * [`R503::Search`]
    pub async fn Wrapper_Verify_Fingerprint(&mut self) -> bool {
        // NOTE: In case of error here, don't set the aura, it's done in the sub functions!

        // =====
        // 1) Verify the password.
        if !self.Wrapper_Setup().await {
            error!("Can't setup scanner");
            return false;
        } else {
            info!("Setup complete");

            // =====
            // 2) Get the fingerprint.
            if !self.Wrapper_Get_Fingerprint(1).await {
                error!("Couldn't scan the finger");
                return false;
            } else {
                info!("Scanned and saved the finger");

                // =====
                // 3) Search for the fingerprint
                // NOTE: This is not a wrapper, so here we need to check the result,
                //       and set the aura accordingly.
                match self.Search(1, 0, 0xffff).await {
                    Status::CmdExecComplete => {
                        info!("Fingerprint found");

                        self.Wrapper_AuraSet_Off().await;
                        return true;
                    }
                    Status::ErrorReceivePackage => {
                        error!("Package receive: Wrapper_Verify_Fingerprint()/Search()");

                        self.Wrapper_AuraSet_BlinkinRedMedium().await;
                        return false;
                    }
                    Status::ErrorNoMatchingFinger => {
                        error!("No matching finger");

                        self.Wrapper_AuraSet_BlinkinRedMedium().await;
                        return false;
                    }
                    stat => {
                        error!("Unknown return code='{=u8:#04x}'", stat as u8);

                        self.Wrapper_AuraSet_Off().await;
                        return false;
                    }
                }
            }
        }
    }
}
