#![no_std]
#![allow(non_snake_case)]	// I want to keep with the manufacturers naming scheme.

use defmt::{debug, info, error, trace};

use embassy_rp::{into_ref, Peripheral};
use embassy_rp::gpio::{AnyPin, Input, Pull, Level}; // For the wakeup.
use embassy_rp::uart::{
    Async, Config, Instance, InterruptHandler,
    Uart, UartTx, UartRx, TxPin, RxPin
};
use embassy_rp::interrupt::typelevel::Binding;
use embassy_rp::dma::Channel;
use embassy_time::{with_timeout, Duration, Timer};

use heapless::Vec;
use core::mem::transmute;

// =====

const START:		u16  = 0xEF01;
const STORE:		u16  = 0x0001;	// 1-127 (high byte front and low byte behind)
const DISABLE_RW:	bool = false;	// Disable the read and write functions.

#[derive(Copy, Clone)]
#[repr(u8)]
pub enum Status {
    CmdExecComplete			= 0x00,
    ErrorReceivePackage			= 0x01,
    ErrorNoFingerOnSensor		= 0x02,
    ErrorEnroleFinger			= 0x03,
    ErrorGenCharFileDistortedImage	= 0x06,
    ErrorGenCharFileSmallImage		= 0x07,
    ErrorNoFingerMatch			= 0x08,
    ErrorNoMatchingFinger		= 0x09,
    ErrorCombineCharFiles		= 0x0a,
    ErrorPageIdBeyondLibrary		= 0x0b,
    ErrorReadingTemplateFromLibrary	= 0x0c,
    ErrorUploadTemplate			= 0x0d,
    ErrorReceiveData			= 0x0e,
    ErrorUploadImage			= 0x0f,
    ErrorDeleteTemplate			= 0x10,
    ErrorClearLibrary			= 0x11,
    ErrorPassword			= 0x13,
    ErrorMissingValidPrimaryImage	= 0x15,
    ErrorWriteFlash			= 0x18,
    ErrorNoDef				= 0x19,
    ErrorInvalidRegister		= 0x1a,
    ErrorIncorrectConfigRegister	= 0x1b,
    ErrorWrongNotepadNumber		= 0x1c,
    ErrorFailedOperateCommunicationPort	= 0x1d,
    ErrorSensorAbnormal			= 0x29,
    ErrorBadPackage			= 0xff
}

// https://www.reddit.com/r/rust/comments/36pgn9/integer_to_enum_after_removal_of_fromprimitive/
impl From<u8> for Status {
    fn from(t:u8) -> Status {
	unsafe { transmute(t) }
    }
}

// These are in Hex order. Further down, they're defined in the order they
// came in the documentation.
#[derive(Copy, Clone)]
#[repr(u8)]
pub enum Command {
    GenImg		= 0x01,
    Img2Tz		= 0x02,
    Match		= 0x03,
    Search		= 0x04,
    RegModel		= 0x05,
    Store		= 0x06,
    LoadChar		= 0x07,
    UpChar		= 0x08,
    DownChar		= 0x09,
    UpImage		= 0x0a,
    DownImage		= 0x0b,
    DeletChar		= 0x0c,
    Empty		= 0x0d,
    SetSysPara		= 0x0e,
    ReadSysPara		= 0x0f,
    SetPwd		= 0x12,
    VfyPwd		= 0x13,
    SetAdder		= 0x15,
    ReadInfPage		= 0x16,
    Control		= 0x17,
    ReadNotepad		= 0x19,
    TempleteNum		= 0x1d,
    GetImageEx		= 0x28,
    Cancel		= 0x30,
    CheckSensor		= 0x36,
    GetAlgVer		= 0x39,
    GetFwVer		= 0x3a,
    SoftRst		= 0x3d,
    HandShake		= 0x40,
    GetRandomCode	= 0x14,
    WriteNotepad	= 0x18,
    ReadIndexTable	= 0x1f,
    AuraLedConfig	= 0x35,
    ReadProdInfo	= 0x3c
}

#[derive(Copy, Clone)]
#[repr(u16)]
pub enum Packets {
    StartCode		= 0xEF01,	// High byte transferred first.
    CommandPacket	= 0x01,
    DataPacket		= 0x02,
    AckPacket		= 0x07,
    EndDataPacket	= 0x08
}

#[derive(Copy, Clone)]
#[repr(u8)]
pub enum PacketCode {
    CommandPacket	= 0x01,
    DataPacket		= 0x02,
    AckPacket		= 0x07,
    DataPackageEnd	= 0x08
}

#[derive(Copy, Clone)]
#[repr(u8)]
pub enum AuroraLEDControl {
    BreathingLight	= 0x01,
    FlashingLight	= 0x02,
    AlwaysOn		= 0x03,
    AlwaysOff		= 0x04,
    GraduallyOn		= 0x05,
    GraduallyOff	= 0x06
}

#[derive(Copy, Clone)]
#[repr(u8)]
pub enum AuroraLEDColour {
    Red			= 0x01,
    Blue		= 0x02,
    Purple		= 0x03
}

// Highly subjective, but..
#[derive(Copy, Clone)]
#[repr(u8)]
pub enum AuroraLEDSpeed {
    Slow		= 0xC8,
    Medium		= 0x20,
    Fast		= 0x02
}

// =====

// T => UART0/UART1
pub struct R503<'l, T: Instance> {
    tx:			UartTx<'l, T, Async>,
    rx:			UartRx<'l, T, Async>,
    wakeup:		Input<'l>,

    pub address:	u32,
    pub password:	u32,
    buffer:		Vec<u8, 128>,
    received:		Vec<u8, 128>,
}

// NOTE: Pins must be consecutive, otherwise it'll segfault!
// Channel => DMA_CH0/DMA_CH1
impl<'l, T: Instance> R503<'l, T> {
    pub fn new(
	uart:			impl Peripheral<P = T> + 'l,
	irqs:			impl Binding<<T as embassy_rp::uart::Instance>::Interrupt, InterruptHandler<T>>,
	pin_send:		impl TxPin<T>,
	pin_send_dma:		impl Peripheral<P = impl Channel> + 'l,
	pin_receive:		impl RxPin<T>,
	pin_receive_dma:	impl Peripheral<P = impl Channel> + 'l,
	pin_wakeup:		AnyPin
    ) -> Self {
	into_ref!(pin_send_dma);

	// Set default passwords.
	let address  = 0xFFFFFFFF;
	let password = 0x00000000;

	// Configure the communication protocol etc.
	let mut config = Config::default(); // => 115200/8N1
	config.baudrate = 57600;

	// Initialize the fingerprint scanner.
	let uart = Uart::new(uart, pin_send, pin_receive, irqs, pin_send_dma, pin_receive_dma, config);
	let (tx, rx) = uart.split();

	// Initialize the WAKEUP pin.
	let wakeup = Input::new(pin_wakeup, Pull::Down);
	match wakeup.get_level() {
	    Level::Low  => debug!("Initial WAKEUP level: LOW"),
	    Level::High => debug!("Initial WAKEUP level: HIGH")
	}

	Self {
	    tx:		tx,
	    rx:		rx,
	    wakeup:	wakeup,
	    address:	address,
	    password:	password,
	    buffer:	heapless::Vec::new(),
	    received:	heapless::Vec::new(),
	}
    }

    // ===== Internal functions

    // Data package format
    // Name		Length		Description
    // ==========================================================================================================
    // Start	2 bytes		Fixed value of 0xEF01; High byte transferred first.
    // Address	4 bytes		Default value is 0xFFFFFFFF, which can be modified by command.
    //				High byte transferred first and at wrong adder value, module
    //				will reject to transfer.
    // PID	1 byte		01H	Command packet;
    //				02H	Data packet; Data packet shall not appear alone in executing
    //					processs, must follow command packet or acknowledge packet.
    //				07H	Acknowledge packet;
    //				08H	End of Data packet.
    // LENGTH	2 bytes		Refers to the length of package content (command packets and data packets)
    //				plus the length of Checksum (2 bytes). Unit is byte. Max length is 256 bytes.
    //				And high byte is transferred first.
    // DATA	-		It can be commands, data, command’s parameters, acknowledge result, etc.
    //				(fingerprint character value, template are all deemed as data);
    // SUM	2 bytes		The arithmetic sum of package identifier, package length and all package
    //				contens. Overflowing bits are omitted. high byte is transferred first.

    async fn write(&mut self) -> u8 {
	debug!("write='{:?}'", self.buffer[..]);
	let _ = self.debug_vec(&self.buffer, true).await;

	if DISABLE_RW {
	    Timer::after_millis(250).await; // Give it quarter of a sec for debug output to catch up.
	    return Status::CmdExecComplete as u8; // Fake a success.
	}

	match self.tx.write(&self.buffer).await {
	    Ok(..) => {
		info!("Write successful.");
		return Status::CmdExecComplete as u8;
	    }
	    Err(e) => {
		error!("Write error: {:?}", e);
		return Status::ErrorReceivePackage as u8;
	    }
	}
    }

    async fn read(&mut self, timeout: u64) -> Vec<u8, 128> {
	info!("Reading reply.");

	let mut buf: [u8; 1] = [0; 1]; // Can only read one byte at a time!
	let mut data: Vec<u8, 128> = heapless::Vec::new(); // Return buffer.
	let mut cnt: u8 = 0; // Keep track of how many packages we've received.

	if DISABLE_RW {
	    // Just for debugging purposes.
	    Timer::after_millis(250).await; // Give it quarter of a sec for debug output to catch up.
	    debug!("  Read disabled by `DISABLE_RW`.");

	    return data; // Fake a success.
	}

	loop {
	    // Read byte.
	    match with_timeout(Duration::from_millis(timeout), self.rx.read(&mut buf)).await {
		Ok(..) => {
		    // Extract and save read byte.
		    trace!("  r({:03})='{=u8:#04x}H' ({:03}D)", cnt, buf[0], buf[0]);
		    let _ = data.push(buf[0]).unwrap();
		}
		Err(..) => break // TimeoutError -> Ignore.
	    }

	    cnt = cnt + 1;
	}
	debug!("read='{:?}'", data[..]);

	if data.len() < 1 {
	    error!("Empty response - no data");
	    return data; // Fake a success.
	}
	info!("Read successful.");

	// Save the response.
	self.received = data.clone();

	return data;
    }

    // -----

    async fn send_command(&mut self, command: Command, data: Vec<u8, 128>) -> Status {
	info!("Sending command {=u8:#04x}H ({:?})", command as u8, self.debug_vec(&data, false).await);

	// Clear buffer.
	self.buffer.clear();

	// Setup data package.
	self.write_cmd_bytes(&START.to_be_bytes()[..]).await;		// Start		u16
	self.write_cmd_bytes(&self.address.to_be_bytes()[..]).await;	// Address		u32
	self.write_cmd_bytes(&[PacketCode::CommandPacket as u8]).await;	// Package identifier	u8

	// Add the length of the package content (command packets and data packets). See below.
	// Length is calculated on 'the Package Identifier (1 byte) + data (??) + checksum (2 bytes)'.
	let len: u16 = (1 + data.len() + 2).try_into().unwrap();
	self.write_cmd_bytes(&len.to_be_bytes()[..]).await;		// Package Length	u16

	// Add the instruction code (command).
	self.write_cmd_bytes(&[command as u8]).await;			// Instruction Code	u8

	// Add the data, if any.
	self.write_cmd_bytes(&data).await;

	// Calculate and add checksum.
	// Checksum is calculated on 'length (2 bytes) + data (??)'.
	let chk = self.compute_checksum().await;
	self.write_cmd_bytes(&chk.to_be_bytes()[..]).await;		// Checksum

	// Send package.
	self.write().await;

	// =====

	// Come commands take longer to start responding. So give them a bit more time.
	// NOTE: Need to update this as I start testing more commands.
	let timeout: u64;
	match command {
	    Command::GenImg	=> timeout =  300,
	    Command::Img2Tz	=> timeout = 1000,
	    _			=> timeout =  200
	}

	// Read response. Will save in `self.received`.
	if self.read(timeout).await.is_empty() {
	    return Status::ErrorReceivePackage;
	};

	// Parse result and return the Status.
	return self.parse_result().await;
    }

    async fn write_cmd_bytes(&mut self, bytes: &[u8]) {
	let _ = self.buffer.extend_from_slice(bytes);
    }

    async fn compute_checksum(&self) -> u16 {
	let mut checksum = 0u16;
	let check_end = self.buffer.len();
	let checked_bytes = &self.buffer[6..check_end];
	for byte in checked_bytes {
	    checksum += (*byte) as u16;
	}
	return checksum;
    }

    async fn parse_result(&mut self) -> Status {
	info!("Parsing reply.");

	if self.received.is_empty() {
	    return Status::ErrorReceivePackage;
	}

	// Known values:
	//   1) Byte  1- 2 should contain the START code	- `0xFE01`.
	//   2) Byte  3- 6 should contain the ADDRESS		- `0xFFFFFFFF`.
	//   3) Byte     7 should contain the PID		- `0x07H` ("Acknowledge packet").
	let start = u16::from_be_bytes([self.received[0], self.received[1]]);
	if start != START {
	    error!("Bad package (start)");
	    return Status::ErrorReceivePackage;
	} else {
	    debug!("  Package start is ok.");
	}

	let address = u32::from_be_bytes([self.received[2], self.received[3], self.received[4], self.received[5]]);
	if self.buffer[9] == Command::SetAdder as u8 &&
	    self.received[6] == 0x07 &&
	    self.received[9] == 0x00 &&
	    address != self.address
	{
	    // Change of address was requested
	    // AND the scanner reported all ok
	    // AND the address returned does not match the one we used initially.
	    // => Change the global address.
	    self.address = address;
	    info!("Address updated");
	} else {
	    if address != self.address {
		error!("Bad package (address)");
		return Status::ErrorReceivePackage;
	    } else {
		debug!("  Package address is ok.");
	    }
	}

	let pid = u8::from_be_bytes([self.received[6]]);
	if pid != 0x07 {
	    error!("Bad package (pid)");
	    return Status::ErrorReceivePackage;
	} else {
	    debug!("  Package pid is ok.");
	}

	// TODO: Depends on DATA returned:
	//   4) Byte  8- 9 should contain the LENGTH		- `0x0003`.
	//   5) Byte    10 should contain the DATA		- `0x00`.
	//                 could also be ConfirmationCode	- `0x00`.
	//   6) Byte 11-12 should contain the SUM		- `0x000a`.

	return self.received[9].into();
    }

    async fn debug_vec(&self, buf: &Vec<u8, 128>, out: bool) -> [u8; 128] {
	let mut a: [u8; 128] = [0; 128];
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

    // ===== System-related instructions

    // Description: Verify Module’s handshaking password.
    // Input Parameter: PassWord (4 bytes)
    // Return Parameter: Confirmation code (1 byte)
    //   Confirmation code = 00H: Correct password;
    //   Confirmation code = 01H: Error when receiving package;
    //   Confirmation code = 13H: Wrong password;
    // Instruction code: 13H
    // Command Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x01
    //   Package Length		 2 byte		0x0007
    //   Instruction code	 1 byte		0x13
    //   Data			 4 bytes
    //     PassWord		 4 byte
    //   Checksum		 2 bytes	Sum		(see top)
    // Acknowledge Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x07
    //   Package Length		 2 byte		0x0003
    //   Confirmation code	 1 byte		xx		(see above)
    //   Checksum		 2 bytes	Sum		(see top)
    pub async fn VfyPwd(&mut self, pass: u32) -> Status {
	info!("COMMAND: Checking password: {=u32:#010x}H", pass);

	let mut data: Vec<u8, 128> = heapless::Vec::new();
	let split: [u8; 4] = pass.to_be_bytes();
	data.extend(split.iter().map(|&i| i));

	return self.send_command(Command::VfyPwd, data).await;
    }

    // Description: Set Module’s handshaking password.
    // Input Parameter: PassWord (4 bytes)
    // Return Parameter: Confirmation code (1 byte)
    //   Confirmation code=00H: password setting complete;
    //   Confirmation code=01H: error when receiving package;
    // Instruction code: 12H
    // Command Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x01
    //   Package Length		 2 byte		0x0007
    //   Instruction code	 1 byte		0x12
    //   Data			 4 bytes
    //     PassWord		 4 byte
    //   Checksum		 2 bytes	Sum		(see top)
    // Acknowledge Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x07
    //   Package Length		 2 byte		0x0003
    //   Confirmation code	 1 byte		xx		(see above)
    //   Checksum		 2 bytes	Sum		(see top)
    pub async fn SetPwd(&mut self, pass: u32) -> Status {
	info!("COMMAND: Setting module password: {=u32:#010x}H", pass);

	let mut data: Vec<u8, 128> = heapless::Vec::new();
	let split: [u8; 4] = pass.to_be_bytes();
	data.extend(split.iter().map(|&i| i));

	match self.send_command(Command::SetPwd, data).await {
	    Status::CmdExecComplete => {
		info!("Password changed");
		self.password = pass;
		return Status::CmdExecComplete;
	    }
	    ret @ _ => return ret
	}
    }

    // Description: Set Module address.
    // Input Parameter: None.
    // Return Parameter: Confirmation code (1 byte)
    //   Confirmation code=00H: address setting complete;
    //   Confirmation code=01H: error when receiving package;
    // Instruction code: 15H
    // Command Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x01
    //   Package Length		 2 byte		0x0007
    //   Instruction code	 1 byte		0x13
    //   Data			 4 bytes
    //     NewAddress		 4 byte
    //   Checksum		 2 bytes	Sum		(see top)
    // Acknowledge Package format:
    //   Header			 2 bytes	0xEF01
    //   NewAddress		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x07
    //   Package Length		 2 byte		0x0007
    //   Confirmation code	 1 byte		xx		(see above)
    //   Checksum		 2 bytes	Sum		(see top)
    pub async fn SetAdder(&mut self, addr: u32) -> Status {
	info!("COMMAND: Setting module address: {=u32:#010x}", addr);

	let mut data: Vec<u8, 128> = heapless::Vec::new();
	let split: [u8; 4] = addr.to_be_bytes();
	data.extend(split.iter().map(|&i| i));

	return self.send_command(Command::SetAdder, data).await;
    }

    // Description: Operation parameter settings.
    // Input Parameter: Parameter number (1 + 1 byte).
    // Return Parameter: Confirmation code (1 byte)
    //   Confirmation code=00H: parameter setting complete;
    //   Confirmation code=01H: error when receiving package;
    //   Confirmation code=1aH: wrong register number;
    // Instruction code: 0eH
    // Command Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x01
    //   Package Length		 2 byte		0x0007
    //   Instruction code	 1 byte		0x0e
    //   Data			 2 bytes
    //     Parameter Number	 1 byte		4/5/6
    //     Content		 1 byte		xx
    //   Checksum		 2 bytes	Sum		(see top)
    // Acknowledge Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x07
    //   Package Length		 2 byte		0x0003
    //   Confirmation code	 1 byte		xx		(see above)
    //   Checksum		 2 bytes	Sum		(see top)
    pub async fn SetSysPara(&mut self, param: u8, content: u8) -> Status {
	info!("COMMAND: Set system parameters: {=u8:#04x}H/{=u8:#04x}H", param, content);

	let mut data: Vec<u8, 128> = heapless::Vec::new();
	let _ = data.push(param);
	let _ = data.push(content);

	return self.send_command(Command::SetSysPara, data).await;
    }

    // Description:
    //   For UART protocol, it control the “on/off” of USB port;
    //   For USB protocol, it control the “on/off” of UART port;
    // Input Parameter: control code (1 byte).
    //   Control code ”0” means turns off the port;
    //   Control code ”1” means turns on the port;
    // Return Parameter: Confirmation code (1 byte)
    //   Confirmation code=00H: Port operation complete;
    //   Confirmation code=01H: error when receiving package;
    //   Confirmation code=1dH: fail to operate the communication port;
    // Instruction code: 17H
    // Command Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x01
    //   Package Length		 2 byte		0x0004
    //   Instruction code	 1 byte		0x17
    //   Data			 1 bytes
    //     ControlCode		 1 byte		0/1
    //   Checksum		 2 bytes	Sum		(see top)
    // Acknowledge Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x07
    //   Package Length		 2 byte		0x0003
    //   Confirmation code	 1 byte		xx		(see above)
    //   Checksum		 2 bytes	Sum		(see top)
    pub async fn Control(&mut self, ctrl: u8) -> Status {
	info!("COMMAND: Control: {=u8:#04x}H", ctrl);

	let mut data: Vec<u8, 128> = heapless::Vec::new();
	let _ = data.push(ctrl);

	return self.send_command(Command::Control, data).await;
    }

    // Description: Read Module’s status register and system basic configuration parameters.
    // Input Parameter: none
    // Return Parameter: Confirmation code (1 byte) + Basic Parameter List (16 bytes)
    //   Confirmation code=00H: read complete;
    //   Confirmation code=01H: error when receiving package;
    // Instruction code: 0fH
    // Command Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x01
    //   Package Length		 2 byte		0x0003
    //   Instruction code	 1 byte		0x0f
    //   Checksum		 2 bytes	Sum		(see top)
    // Acknowledge Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x07
    //   Package Length		 2 byte		3+16
    //   Confirmation code	 1 byte		xx		(see above)
    //   Data			16 bytes
    //     Basic Param List	16 byte
    //   Checksum		 2 bytes	Sum		(see top)
    // TODO: Return `Status` and ... (16 bytes - `[u8, 16]`)??
    pub async fn ReadSysPara(&mut self) -> Status {
	info!("COMMAND: Read status register and basic configuration parameters.");

	let data: Vec<u8, 128> = heapless::Vec::new();

	return self.send_command(Command::ReadSysPara, data).await;
    }

    // Description: read the current valid template number of the Module.
    // Input Parameter: none
    // Return Parameter: Confirmation code (1 byte) + Template number (2 bytes)
    //   Confirmation code=0x00: read success;
    //   Confirmation code=0x01: error when receiving package;
    // Instruction code: 1dH
    // Command Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x01
    //   Package Length		 2 byte		0x0003
    //   Instruction code	 1 byte		0x1d
    //   Checksum		 2 bytes	0x0021
    // Acknowledge Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x07
    //   Package Length		 2 byte		0x0005
    //   Confirmation code	 1 byte		xx		(see above)
    //   Data			 2 bytes
    //     Template Number	 2 byte
    //   Checksum		 2 bytes	Sum		(see top)
    // TODO: Return `Status` and ... (2 bytes - `[u8, 2]`)??
    pub async fn TempleteNum(&mut self) -> Status {
	info!("COMMAND: Read current valid template number.");

	let data: Vec<u8, 128> = heapless::Vec::new();

	return self.send_command(Command::TempleteNum, data).await;
    }

    // Description: Read the fingerprint template index table of the module,
    //              read the index table of the fingerprint template up to 256 at a time (32 bytes).
    // Input Parameter: Index page (1 byte)
    //   Index tables are read per page, 256 templates per page
    //   Index page 0 means to read 0 ~ 255 fingerprint template index table;
    //   Index page 1 means to read 256 ~ 511 fingerprint template index table;
    //   Index page 2 means to read 512 ~ 767 fingerprint template index table;
    //   Index page 3 means to read 768 ~ 1023 fingerprint template index table
    // Return Parameter: Confirmation code (1 byte) + Fingerprint Template Index Table (32 bytes)
    //   Confirmation code=0x00: read complete;
    //   Confirmation code=0x01: error when receiving package;
    // Instruction code: 1fH
    // Command Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x01
    //   Package Length		 2 byte		0x0004
    //   Instruction code	 1 byte		0x1f
    //   Data			 1 bytes
    //     Index Page		 1 byte
    //   Checksum		 2 bytes	Sum		(see top)
    // Acknowledge Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x07
    //   Package Length		 2 byte		0x0023
    //   Confirmation code	 1 byte		xx		(see above)
    //   Data			32 bytes
    //     Index Page		32 bytes			(see documentation)
    //   Checksum		 2 bytes	Sum		(see top)
    // TODO: Return `Status` and ... (32 bytes - `[u8, 32]`)??
    pub async fn ReadIndexTable(&mut self, page: u8) -> Status {
	info!("COMMAND: Read fingerprint template index table: {=u8:#04x}H", page);

	let mut data: Vec<u8, 128> = heapless::Vec::new();
	let _ = data.push(page);

	return self.send_command(Command::ReadIndexTable, data).await;
    }

    // ===== Fingerprint-processing instructions

    // Description: Detecting finger and store the detected finger image in ImageBuffer while returning
    //              successfull confirmation code; If there is no finger, returned confirmation code
    //              would be “can’t detect finger”.
    // Input Parameter: none
    // Return Parameter: Confirmation code (1 byte)
    //   Confirmation code=00H: finger collection successs;
    //   Confirmation code=01H: error when receiving package;
    //   Confirmation code=02H: can’t detect finger;
    //   Confirmation code=03H: fail to collect finger;
    // Instruction code: 01H
    // Command Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x01
    //   Package Length		 2 byte		0x0003
    //   Instruction code	 1 byte		0x01
    //   Checksum		 2 bytes	0x05
    // Acknowledge Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x07
    //   Package Length		 2 byte		0x0003
    //   Confirmation code	 1 byte		xx		(see above)
    //   Checksum		 2 bytes	Sum		(see top)
    pub async fn GenImg(&mut self) -> Status {
	info!("COMMAND: Scanning finger.");

	let data: Vec<u8, 128> = heapless::Vec::new();

	return self.send_command(Command::GenImg, data).await;
    }

    // Description: to upload the image in Img_Buffer to upper computer.
    // Input Parameter: none
    // Return Parameter: Confirmation code (1 byte)
    //   Confirmation code=00H: ready to transfer the following data packet;
    //   Confirmation code=01H: error when receiving package;
    //   Confirmation code=0fH: fail to transfer the following data packet;
    // Instruction code: 0aH
    // Command Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x01
    //   Package Length		 2 byte		0x0003
    //   Instruction code	 1 byte		0x0a
    //   Checksum		 2 bytes	0x000e
    // Acknowledge Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x07
    //   Package Length		 2 byte		0x0003
    //   Confirmation code	 1 byte		xx		(see above)
    //   Checksum		 2 bytes	Sum		(see top)
    pub async fn UpImage(&mut self) -> Status {
	info!("COMMAND: Upload image from image buffer to upper computer.");

	let data: Vec<u8, 128> = heapless::Vec::new();

	return self.send_command(Command::UpImage, data).await;
    }

    // Description: Download image from upper computer to Img_Buffer.
    // Input Parameter: none
    // Return Parameter: Confirmation code (1 byte)
    //   Confirmation code=00H: ready to transfer the following data packet;
    //   Confirmation code=01H: error when receiving package;
    //   Confirmation code=0eH: fail to transfer the following data packet;
    // Instruction code: 0bH
    // Command Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x01
    //   Package Length		 2 byte		0x0003
    //   Instruction code	 1 byte		0x0b
    //   Checksum		 2 bytes	0x000f
    // Acknowledge Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x07
    //   Package Length		 2 byte		0x0003
    //   Confirmation code	 1 byte		xx		(see above)
    //   Checksum		 2 bytes	Sum		(see top)
    pub async fn DownImage(&mut self) -> Status {
	info!("COMMAND: Download image from upper computer to image buffer.");

	let data: Vec<u8, 128> = heapless::Vec::new();

	return self.send_command(Command::DownImage, data).await;
    }

    // Description: Generate character file from the original finger image in ImageBuffer and store the
    //              file in CharBuffer1 or CharBuffer2.
    //              Note: BufferID of CharBuffer1 and CharBuffer2 are 1h and 2h respectively. Other values
    //                    (except 1h, 2h) would be processed as CharBuffer2.
    // Input Parameter: (1 byte)
    //   BufferID - character file buffer number (1 byte).
    // Return Parameter: Confirmation code (1 byte)
    //   Confirmation code=00H: generate character file complete;
    //   Confirmation code=01H: error when receiving package;
    //   Confirmation code=06H: fail to generate character file due to the over-disorderly fingerprint image;
    //   Confirmation code=07H: fail to generate character file due to lackness of character point or
    //                          over-smallness of fingerprint image;
    //   Confirmation code=15H: fail to generate the image for the lackness of valid primary image;
    // Instruction code: 02H
    // Command Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x01
    //   Package Length		 2 byte		0x0004
    //   Instruction code	 1 byte		0x02
    //   Data			 1 bytes
    //     BufferID		 1 byte		1|2
    //   Checksum		 2 bytes	Sum		(see top)
    // Acknowledge Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x07
    //   Package Length		 2 byte		0x0003
    //   Confirmation code	 1 byte		xx		(see above)
    //   Checksum		 2 bytes	Sum		(see top)
    pub async fn Img2Tz(&mut self, buff: u8) -> Status {
	info!("COMMAND: Generating character file from finger image: {=u8:#04x}H", buff);

	let mut data: Vec<u8, 128> = heapless::Vec::new();
	let _ = data.push(buff);

	return self.send_command(Command::Img2Tz, data).await;
    }

    // Description: Combine information of character files from CharBuffer1 and CharBuffer2 and generate
    //              a template which is stored back in both CharBuffer1 and CharBuffer2.
    // Input Parameter: none
    // Return Parameter: Confirmation code (1 byte)
    //   Confirmation code=00H: operation success;
    //   Confirmation code=01H: error when receiving package;
    //   Confirmation code=0aH: fail to combine the character files. That’s, the character files don’t belong
    //                          to one finger.
    // Instruction code: 05H
    // Command Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x01
    //   Package Length		 2 byte		0x0003
    //   Instruction code	 1 byte		0x05
    //   Checksum		 2 bytes	0x09
    // Acknowledge Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x07
    //   Package Length		 2 byte		0x0003
    //   Confirmation code	 1 byte		xx		(see above)
    //   Checksum		 2 bytes	Sum		(see top)
    pub async fn RegModel(&mut self) -> Status {
	info!("COMMAND: Generate fingerprint template.");

	let data: Vec<u8, 128> = heapless::Vec::new();

	return self.send_command(Command::RegModel, data).await;
    }

    // Description: Upload the character file or template of CharBuffer1/CharBuffer2 to upper computer.
    //              Note: BufferID of CharBuffer1 and CharBuffer2 are 1h and 2h respectively. Other values
    //                    (except 1h, 2h) would be processed as CharBuffer2.
    // Input Parameter: BufferID - buffer number (1 byte).
    // Return Parameter: Confirmation code (1 byte)
    //   Confirmation code=00H: ready to transfer the following data packet;
    //   Confirmation code=01H: error when receiving package;
    //   Confirmation code=0dH: error when uploading template;
    // Instruction code: 08H
    // Command Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x01
    //   Package Length		 2 byte		0x0004
    //   Instruction code	 1 byte		0x08
    //   Data			 1 bytes
    //     BufferID		 1 byte		
    //   Checksum		 2 bytes	Sum		(see top)
    // Acknowledge Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x07
    //   Package Length		 2 byte		0x0003
    //   Confirmation code	 1 byte		xx		(see above)
    //   Checksum		 2 bytes	Sum		(see top)
    pub async fn UpChar(&mut self, buff: u8) -> Status {
	info!("COMMAND: Upload character file to upper computer: {=u8:#04x}H", buff);

	let mut data: Vec<u8, 128> = heapless::Vec::new();
	let _ = data.push(buff);

	return self.send_command(Command::UpChar, data).await;
    }

    // Description: Upper computer download template to module buffer.
    // Input Parameter: CharBufferID - buffer number (1 byte).
    // Return Parameter: Confirmation code (1 byte)
    //   Confirmation code=00H: ready to transfer the following data packet;
    //   Confirmation code=01H: error when receiving package;
    //   Confirmation code=0eH: can not receive the following data packet;
    // Instruction code: 09H
    // Command Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x01
    //   Package Length		 2 byte		0x0004
    //   Instruction code	 1 byte		0x09
    //   Data			 1 bytes
    //     CharBufferID		 1 byte		
    //   Checksum		 2 bytes	Sum		(see top)
    // Acknowledge Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x07
    //   Package Length		 2 byte		0x0003
    //   Confirmation code	 1 byte		xx		(see above)
    //   Checksum		 2 bytes	Sum		(see top)
    pub async fn DownChar(&mut self, buff: u8) -> Status {
	info!("COMMAND: Download template to model buffer: {=u8:#04x}H", buff);

	let mut data: Vec<u8, 128> = heapless::Vec::new();
	let _ = data.push(buff);

	return self.send_command(Command::DownChar, data).await;
    }

    // Description: Store the template of specified buffer (Buffer1/Buffer2) at the designated location
    //              of Flash library.
    //              Note: BufferID of CharBuffer1 and CharBuffer2 are 1h and 2h respectively. Other values
    //                    (except 1h, 2h) would be processed as CharBuffer2.
    // Input Parameter: (1 + 2 bytes)
    //   BufferID - buffer number (1 byte);
    //   PageID - flash location of the template, two bytes with high byte front and low byte behind (2 bytes)
    // Return Parameter: Confirmation code (1 byte)
    //   Confirmation code=00H: storage success;
    //   Confirmation code=01H: error when receiving package;
    //   Confirmation code=0bH: addressing PageID is beyond the finger library;
    //   Confirmation code=18H: error when writing Flash;
    // Instruction code: 06H
    // Command Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x01
    //   Package Length		 2 byte		0x0006
    //   Instruction code	 1 byte		0x06
    //   Data			 3 bytes
    //     BufferID		 1 byte
    //     PageID		 2 bytes
    //   Checksum		 2 bytes	Sum		(see top)
    // Acknowledge Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x07
    //   Package Length		 2 byte		0x0003
    //   Confirmation code	 1 byte		xx		(see above)
    //   Checksum		 2 bytes	Sum		(see top)
    pub async fn Store(&mut self, buff: u8, page: u16) -> Status {
	info!("COMMAND: Store fingerprint template in flash: {=u8:#04x}H/{=u16:#06x}H", buff, page);

	let mut data: Vec<u8, 128> = heapless::Vec::new();
	let _ = data.push(buff);

	let split: [u8; 2] = page.to_be_bytes();
	let _ = data.extend(split.iter().map(|&i| i));

	return self.send_command(Command::Store, data).await;
    }

    // Description: Load template at the specified location (PageID) of Flash library to template buffer
    //              CharBuffer1/CharBuffer2
    // Input Parameter: (1 + 2 bytes).
    //   BufferID - buffer number (1 byte);
    //   PageID - flash location of the template, two bytes with high byte front and low byte behind (2 bytes).
    // Return Parameter: Confirmation code (1 byte)
    //   Confirmation code=00H: load success;
    //   Confirmation code=01H: error when receiving package;
    //   Confirmation code=0cH: error when reading template from library or the readout template is invalid;
    //   Confirmation code=0bH: addressing PageID is beyond the finger library;
    // Instruction code: 07H
    // Command Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x01
    //   Package Length		 2 byte		0x0006
    //   Instruction code	 1 byte		0x07
    //   Data			 3 bytes
    //     BufferID		 1 byte
    //     PageID		 2 bytes
    //   Checksum		 2 bytes	Sum		(see top)
    // Acknowledge Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x07
    //   Package Length		 2 byte		0x0003
    //   Confirmation code	 1 byte		xx		(see above)
    //   Checksum		 2 bytes	Sum		(see top)
    pub async fn LoadChar(&mut self, buff: u8, page: u16) -> Status {
	info!("COMMAND: Load template from flash: {=u8:#04x}H/{=u16:#06x}H", buff, page);

	let mut data: Vec<u8, 128> = heapless::Vec::new();
	let _ = data.push(buff);

	let split: [u8; 2] = page.to_be_bytes();
	let _ = data.extend(split.iter().map(|&i| i));

	return self.send_command(Command::LoadChar, data).await;
    }

    // Description: Delete a segment (N) of templates of Flash library started from the specified location
    //              (or PageID);
    // Input Parameter: (2 + 2 bytes)
    //   PageID - template number in flash (2 bytes).
    //   N - number of templates to be deleted (2 bytes).
    // Return Parameter: Confirmation code (1 byte)
    //   Confirmation code=00H: delete success;
    //   Confirmation code=01H: error when receiving package;
    //   Confirmation code=10H: faile to delete templates;
    // Instruction code: 0cH
    // Command Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x01
    //   Package Length		 2 byte		0x0007
    //   Instruction code	 1 byte		0x0c
    //   Data			 4 bytes
    //     PageID		 2 byte
    //     N			 2 bytes
    //   Checksum		 2 bytes	Sum		(see top)
    // Acknowledge Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x07
    //   Package Length		 2 byte		0x0003
    //   Confirmation code	 1 byte		xx		(see above)
    //   Checksum		 2 bytes	Sum		(see top)
    pub async fn DeletChar(&mut self, page: u16, n: u16) -> Status {
	info!("COMMAND: Delete a segment of templates in flash: {=u16:#06x}H/{=u16:#06x}H", page, n);

	let mut data: Vec<u8, 128> = heapless::Vec::new();

	let split_page: [u8; 2] = page.to_be_bytes();
	data.extend(split_page.iter().map(|&i| i));

	let split_n: [u8; 2] = page.to_be_bytes();
	data.extend(split_n.iter().map(|&i| i));

	return self.send_command(Command::DeletChar, data).await;
    }

    // Description: to delete all the templates in the Flash library.
    // Input Parameter: none
    // Return Parameter: Confirmation code (1 byte)
    //   Confirmation code=00H: empty success;
    //   Confirmation code=01H: error when receiving package;
    //   Confirmation code=11H: fail to clear finger library;
    // Instruction code: 0dH
    // Command Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x01
    //   Package Length		 2 byte		0x0003
    //   Instruction code	 1 byte		0x0d
    //   Checksum		 2 bytes	0x0011
    // Acknowledge Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x07
    //   Package Length		 2 byte		0x0003
    //   Confirmation code	 1 byte		xx		(see above)
    //   Checksum		 2 bytes	Sum		(see top)
    pub async fn Empty(&mut self) -> Status {
	info!("COMMAND: Delete all templates in flash.");

	let data: Vec<u8, 128> = heapless::Vec::new();

	return self.send_command(Command::Empty, data).await;
    }

    // Description: Carry out precise matching of templates from CharBuffer1 and CharBuffer2, providing
    //              matching results.
    // Input Parameter: none
    // Return Parameter: Confirmation code (1 byte)，matching score.
    //   Confirmation code=00H: templates of the two buffers are matching;
    //   Confirmation code=01H: error when receiving package;
    //   Confirmation code=08H: templates of the two buffers aren’t matching;
    // Instruction code: 03H
    // Command Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x01
    //   Package Length		 2 byte		0x0003
    //   Instruction code	 1 byte		0x03
    //   Checksum		 2 bytes	0x07
    // Acknowledge Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x07
    //   Package Length		 2 byte		0x0005
    //   Confirmation code	 1 byte		xx		(see above)
    //   Data			 2 bytes
    //     Matching Score	 2 byte
    //   Checksum		 2 bytes	Sum		(see top)
    pub async fn Match(&mut self) -> Status {
	info!("COMMAND: Match template.");

	let data: Vec<u8, 128> = heapless::Vec::new();

	return self.send_command(Command::Match, data).await;
    }

    // Description: Search the whole finger library for the template that matches the one in CharBuffer1
    //              or CharBuffer2. When found, PageID will be returned.
    // Input Parameter: (1 + 2 + 2 bytes).
    //   BufferID - character file buffer number (1 byte).
    //   StartPage - searching start address (2 bytes).
    //   PageNum - searching numbers (2 bytes)
    // Return Parameter:
    //   Confirmation code (1 byte).
    //     Confirmation code=00H: found the matching finer;
    //     Confirmation code=01H: error when receiving package;
    //     Confirmation code=09H: No matching in the library (both the PageID and matching score are 0);
    //   PageID - matching templates location (2 bytes).
    //   MatchScore (2 bytes).
    // Instruction code: 04H
    // Command Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x01
    //   Package Length		 2 byte		0x0008
    //   Instruction code	 1 byte		0x04
    //   Data			 5 bytes
    //     BufferID		 1 byte
    //     StartPage		 2 bytes
    //     PageNum		 2 bytes
    //   Checksum		 2 bytes	Sum		(see top)
    // Acknowledge Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x07
    //   Package Length		 2 byte		0x0007
    //   Confirmation code	 1 byte		xx		(see above)
    //   Data			 4 bytes
    //     PageID		 2 bytes
    //     MatchScore		 2 bytes
    //   Checksum		 2 bytes	Sum		(see top)
    // TODO: Return `Status`, PageID (2 bytes - `[u8, 2]`) and MatchScore (2 bytes - `[u8, 2]`)??
    pub async fn Search(&mut self, buff: u8, start: u16, page: u16) -> Status {
	info!("COMMAND: Search fingerpringt library for template: {=u8:#04x}H/{=u16:#06x}H/{=u16:#06x}H",
	buff, start, page);

	let mut data: Vec<u8, 128> = heapless::Vec::new();
	let _ = data.push(buff);

	let split_start: [u8; 2] = start.to_be_bytes();
	let _ = data.extend(split_start.iter().map(|&i| i));

	let split_page: [u8; 2] = page.to_be_bytes();
	let _ = data.extend(split_page.iter().map(|&i| i));

	return self.send_command(Command::Search, data).await;
    }

    // Description: Detect the finger, record the fingerprint image and store it in ImageBuffer, return
    //              it and record the successful confirmation code;
    //              If no finger is detected, return no finger confirmation code(the module responds
    //              quickly to each instruction,therefore, for continuous detection, cycle processing
    //              is required, which can be limited to the number of cycles or the total time).
    //              Differences between GetImageEx and the GetImage:
    //                GetImage: Return the confirmation code 0x00 when the image quality is too bad
    //                          (image collection succeeded).
    //                GetImageEx: Return the confirmation code 0x07 when the image quality is too bad
    //                            (poor collection quality).
    // Input Parameter: none
    // Return Parameter: Confirmation code (1 byte).
    //   Confirmation code=0x00: read success
    //   Confirmation code=0x01: error when receiving package;
    //   Confirmation code=0x02: no fingers on the sensor;
    //   Confirmation code=0x03: unsuccessful entry;
    //   Confirmation code=0x07: poor image quality;
    // Instruction code: 28H
    // Command Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x01
    //   Package Length		 2 byte		0x0003
    //   Instruction code	 1 byte		0x28
    //   Checksum		 2 bytes	Sum		(see top)
    // Acknowledge Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x07
    //   Package Length		 2 byte		0x0003
    //   Confirmation code	 1 byte		xx		(see above)
    //   Checksum		 2 bytes	Sum		(see top)
    pub async fn GetImageEx(&mut self) -> Status {
	info!("COMMAND: Scan finger, record image and store it buffer.");

	let data: Vec<u8, 128> = heapless::Vec::new();

	return self.send_command(Command::GetImageEx, data).await;
    }

    // Description: Cancel instruction
    // Input Parameter: none
    // Return Parameter: Confirmation code (1 byte).
    //   Confirmation code=0x00: cancel setting successful;
    //   Confirmation code=other: cancel setting failed;
    // Instruction code: 30H
    // Command Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x01
    //   Package Length		 2 byte		0x0003
    //   Instruction code	 1 byte		0x30
    //   Checksum		 2 bytes	Sum		(see top)
    // Acknowledge Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x07
    //   Package Length		 2 byte		0x0003
    //   Confirmation code	 1 byte		xx		(see above)
    //   Checksum		 2 bytes	Sum		(see top)
    pub async fn Cancel(&mut self) -> Status {
	info!("COMMAND: Cancel instruction.");

	let data: Vec<u8, 128> = heapless::Vec::new();

	return self.send_command(Command::Cancel, data).await;
    }

    // Description: Send handshake instructions to the module. If the module works normally, the
    //              confirmation code 0x00 will be returned. The upper computer can continue to
    //              send instructions to the module.If the confirmation code is other or no reply,
    //              it means that the device is abnormal.
    // Input Parameter: none
    // Return Parameter: Confirmation code (1 byte).
    //   Confirmation code=0x00: the device is normal and can receive instructions;
    //   Confirmation code=other: the device is abnormal;
    // Instruction code: 40H
    // Command Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x01
    //   Package Length		 2 byte		0x0003
    //   Instruction code	 1 byte		0x40
    //   Checksum		 2 bytes	Sum		(see top)
    // Acknowledge Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x07
    //   Package Length		 2 byte		0x0003
    //   Confirmation code	 1 byte		xx		(see above)
    //   Checksum		 2 bytes	Sum		(see top)
    pub async fn HandShake(&mut self) -> Status {
	info!("COMMAND: Handshake.");

	let data: Vec<u8, 128> = heapless::Vec::new();

	return self.send_command(Command::HandShake, data).await;
    }

    // Description: Check whether the sensor is normal.
    // Input Parameter: none
    // Return Parameter: Confirmation code (1 byte).
    //   Confirmation code=0x00: the sensor is normal;
    //   Confirmation code=0x29: the sensor is abnormal;
    // Instruction code: 36H
    // Command Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x01
    //   Package Length		 2 byte		0x0003
    //   Instruction code	 1 byte		0x36
    //   Checksum		 2 bytes	Sum		(see top)
    // Acknowledge Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x07
    //   Package Length		 2 byte		0x0003
    //   Confirmation code	 1 byte		xx		(see above)
    //   Checksum		 2 bytes	Sum		(see top)
    pub async fn CheckSensor(&mut self) -> Status {
	info!("COMMAND: Checking sensor.");

	let data: Vec<u8, 128> = heapless::Vec::new();

	return self.send_command(Command::CheckSensor, data).await;
    }

    // Description: Get the algorithm library version.
    // Input Parameter: none
    // Return Parameter:
    //   Confirmation code (1 byte).
    //     Confirmation code=0x00: success;
    //     Confirmation code=0x01: error when receiving package;
    //   AlgVer - algorithm library version string (32 bytes).
    // Instruction code: 39H
    // Command Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x01
    //   Package Length		 2 byte		0x0003
    //   Instruction code	 1 byte		0x39
    //   Checksum		 2 bytes	Sum		(see top)
    // Acknowledge Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x07
    //   Package Length		 2 byte		0x0023
    //   Confirmation code	 1 byte		xx		(see above)
    //   Data			32 bytes
    //     AlgVer		32 bytes
    //   Checksum		 2 bytes	Sum		(see top)
    // TODO: Return `Status` and AlgVer (32 bytes - `[u8, 32]`)??
    pub async fn GetAlgVer(&mut self) -> Status {
	info!("COMMAND: Get algorithm library version.");

	let data: Vec<u8, 128> = heapless::Vec::new();

	return self.send_command(Command::GetAlgVer, data).await;
    }

    // Description: Get the firmware version.
    // Input Parameter: none
    // Return Parameter:
    //   Confirmation code (1 byte).
    //     Confirmation code=0x00: success;
    //     Confirmation code=0x01: error when receiving package;
    //   FwVer - firmware version string (32 bytes).
    // Instruction code: 3aH
    // Command Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x01
    //   Package Length		 2 byte		0x0003
    //   Instruction code	 1 byte		0x3a
    //   Checksum		 2 bytes	Sum		(see top)
    // Acknowledge Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x07
    //   Package Length		 2 byte		0x0023
    //   Confirmation code	 1 byte		xx		(see above)
    //   Data			32 bytes
    //     FwVer		32 bytes
    //   Checksum		 2 bytes	Sum		(see top)
    // TODO: Return `Status` and FwVer (32 bytes - `[u8, 32]`)??
    pub async fn GetFwVer(&mut self) -> Status {
	info!("COMMAND: Get firmware version.");

	let data: Vec<u8, 128> = heapless::Vec::new();

	return self.send_command(Command::GetFwVer, data).await;
    }

    // Description: Read product information.
    // Input Parameter: none
    // Return Parameter:
    //   Confirmation code (1 byte).
    //     Confirmation code=0x00: success;
    //     Confirmation code=0x01: error when receiving package;
    //   ProdInfo - product information (46 bytes).
    // Instruction code: 3cH
    // Command Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x01
    //   Package Length		 2 byte		0x0003
    //   Instruction code	 1 byte		0x3c
    //   Checksum		 2 bytes	Sum		(see top)
    // Acknowledge Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x07
    //   Package Length		 2 byte		0x0031
    //   Confirmation code	 1 byte		xx		(see above)
    //   Data			46 bytes
    //     ProdInfo		46 bytes			(see documentation)
    //   Checksum		 2 bytes	Sum		(see top)
    // TODO: Return `Status` and ProdInfo (46 bytes - `[u8, 46]`)??
    pub async fn ReadProdInfo(&mut self) -> Status {
	info!("COMMAND: Read product information.");

	let data: Vec<u8, 128> = heapless::Vec::new();

	return self.send_command(Command::ReadProdInfo, data).await;
    }

    // Description: Send soft reset instruction to the module. If the module works normally, return
    //              confirmation code 0x00, and then perform reset operation.
    // Input Parameter: none
    // Return Parameter: Confirmation code (1 byte).
    //   Confirmation code=0x00: success;
    //   Confirmation code=other: device is abnormal
    // Instruction code: 3dH
    // Command Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x01
    //   Package Length		 2 byte		0x0003
    //   Instruction code	 1 byte		0x3d
    //   Checksum		 2 bytes	Sum		(see top)
    // Acknowledge Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x07
    //   Package Length		 2 byte		0x0003
    //   Confirmation code	 1 byte		xx		(see above)
    //   Checksum		 2 bytes	Sum		(see top)
    pub async fn SoftRst(&mut self) -> Status {
	info!("COMMAND: Soft reset.");

	let data: Vec<u8, 128> = heapless::Vec::new();

	return self.send_command(Command::SoftRst, data).await;
    }

    // Description: Aura LED control
    // Input Parameter: (1 + 1 + 1 + 1 byte)
    //   Control (1 byte).
    //     0x01: Breathing light
    //     0x02: Flashing light
    //     0x03: Light Always on
    //     0x04: Light Always off
    //     0x05: Light gradually on
    //     0x06: Light gradually off
    //   Speed (1 byte).
    //     0x00-0xff, 256 gears, Minimum 5s cycle.
    //   ColorIndex (1 byte).
    //     0x01: Red
    //     0x02: Blue
    //     0x03: Purple
    //   Times (1 byte).
    //     Number of cycles: 0- infinite, 1-255.
    // Return Parameter: Confirmation code (1 byte).
    //   Confirmation code=0x00: success;
    //   Confirmation code=0x01: error when receiving package;
    // Instruction code: 35H
    // Command Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x01
    //   Package Length		 2 byte		0x0007
    //   Instruction code	 1 byte		0x35
    //   Data			 4 bytes
    //     Control code		 1 byte		Ctrl		(see above)
    //     Speed		 1 byte		Speed		(see above)
    //     Colour index		 1 byte		ColourIndex	(see above)
    //     Times		 1 byte		Times		(see above)
    //   Checksum		 2 bytes	Sum		(see top)
    // Acknowledge Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x07
    //   Package Length		 2 byte		0x0003
    //   Confirmation code	 1 byte		xx		(see above)
    //   Checksum		 2 bytes	Sum		(see top)
    pub async fn AuraLedConfig(&mut self, ctrl: AuroraLEDControl, speed: u8, colour: AuroraLEDColour, times: u8)
			       -> Status
    {
	info!("COMMAND: Setting up aura LED: {=u8:#04x}H/{=u8:#04x}H/{=u8:#04x}H/{=u8:#04x}H",
	ctrl as u8, speed, colour as u8, times);

	let mut data: Vec<u8, 128> = heapless::Vec::new();
	let _ = data.push(ctrl as u8);
	let _ = data.push(speed);
	let _ = data.push(colour as u8);
	let _ = data.push(times);

	return self.send_command(Command::AuraLedConfig, data).await;
    }

    // ===== Other instructions

    // Description: Command the Module to generate a random number and return it to upper computer.
    // Input Parameter: none
    // Return Parameter: Confirmation code (1 byte).
    //   Confirmation code=00H: generation success;
    //   Confirmation code=01H: error when receiving package;
    // Instruction code: 14H
    // Command Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x01
    //   Package Length		 2 byte		0x0003
    //   Instruction code	 1 byte		0x14
    //   Checksum		 2 bytes	0x0018
    // Acknowledge Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x07
    //   Package Length		 2 byte		0x0007
    //   Confirmation code	 1 byte		xx		(see above)
    //   Data			 4 bytes
    //     Random Number	 4 bytes
    //   Checksum		 2 bytes	Sum		(see top)
    pub async fn GetRandomCode(&mut self) -> Status {
	info!("COMMAND: Get random code.");

	let data: Vec<u8, 128> = heapless::Vec::new();

	return self.send_command(Command::GetRandomCode, data).await;
    }

    // Description: read information page(512bytes)
    // Input Parameter: none
    // Return Parameter: Confirmation code (1 byte).
    //   Confirmation code=00H: ready to transfer the following data packet;
    //   Confirmation code=01H: error when receiving package;
    //   Confirmation code=0fH: can not transfer the following data packet;
    // Instruction code: 16H
    // Command Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x01
    //   Package Length		 2 byte		0x0003
    //   Instruction code	 1 byte		0x16
    //   Checksum		 2 bytes	Sum		(see top)
    // Acknowledge Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x07
    //   Package Length		 2 byte		0x0003
    //   Confirmation code	 1 byte		xx		(see above)
    //   Checksum		 2 bytes	Sum		(see top)
    pub async fn ReadInfPage(&mut self) -> Status {
	info!("COMMAND: Read information page.");

	let data: Vec<u8, 128> = heapless::Vec::new();

	return self.send_command(Command::ReadInfPage, data).await;
    }

    // Description: Upper computer to write data to the specified Flash page. Also see ReadNotepad.
    // Input Parameter:
    //   PageNumber - notepad page number (1 byte).
    //   Content - data (32 bytes).
    // Return Parameter: Confirmation code (1 byte)
    //   Confirmation code=00H: write success;
    //   Confirmation code=01H: error when receiving package;
    // Instruction code: 18H
    // Command Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x01
    //   Package Length		 2 byte		0x0036
    //   Instruction code	 1 byte		0x18
    //   Data			33 bytes
    //     PageNumber		 1 byte
    //     Content		32 bytes
    //   Checksum		 2 bytes	Sum		(see top)
    // Acknowledge Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x07
    //   Package Length		 2 byte		0x0003
    //   Confirmation code	 1 byte		xx		(see above)
    //   Checksum		 2 bytes	Sum		(see top)
    pub async fn WriteNotepad(&mut self, page: u8, content: &[u8; 32]) -> Status {
	info!("COMMAND: Write notepad: {=u8:#04x}H/<content>", page); // Not sure how to output a `&[u128; 2]`.

	let mut data: Vec<u8, 128> = heapless::Vec::new();
	let _ = data.push(page);
	let _ = data.extend(content.iter().map(|&i| i));

	return self.send_command(Command::WriteNotepad, data).await;
    }

    // Description: Read the specified page’s data content. Also see WriteNotepad.
    // Input Parameter: none
    // Return Parameter:
    //   Confirmation code (1 byte).
    //     Confirmation code=00H: read success;
    //     Confirmation code=01H: error when receiving package;
    //   Data content (32 bytes).
    // Instruction code: 19H
    // Command Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x01
    //   Package Length		 2 byte		0x0004
    //   Instruction code	 1 byte		0x19
    //   Data			 1 bytes
    //     PageNumber		 1 byte
    //   Checksum		 2 bytes	Sum		(see top)
    // Acknowledge Package format:
    //   Header			 2 bytes	0xEF01
    //   Address		 4 bytes	xxxxxx
    //   Package Identifier	 1 byte		0x07
    //   Package Length		 2 byte		3+32
    //   Confirmation code	 1 byte		xx		(see above)
    //   Data			32 bytes
    //     User Content		32 bytes
    //   Checksum		 2 bytes	Sum		(see top)
    // TODO: Return `Status` and User Content (32 bytes - `[u8, 32]`)??
    pub async fn ReadNotepad(&mut self) -> Status {
	info!("COMMAND: Read notepad.");

	let data: Vec<u8, 128> = heapless::Vec::new();

	return self.send_command(Command::ReadNotepad, data).await;
    }

    // ===== Wrapper functions
    // Just to simplify life a little. Should just return `true` (failure) or `false` (success).

    pub async fn Wrapper_AuraSet_BlinkinRedSlow(&mut self) -> bool {
	match self.AuraLedConfig(AuroraLEDControl::BreathingLight, AuroraLEDSpeed::Slow as u8,
				 AuroraLEDColour::Red, 0).await
	{
	    Status::CmdExecComplete => {
		info!("Fingerprint scanner LED set to blinking red, slow");

		return false;
	    },
	    Status::ErrorReceivePackage => {
		error!("Fingerprint scanner LED set - package receive");
	    },
	    stat => {
		error!("Unknown return code='{=u8:#04x}'", stat as u8);
	    }
	}

	return true;
    }

    pub async fn Wrapper_AuraSet_BlinkinRedMedium(&mut self) -> bool {
	match self.AuraLedConfig(AuroraLEDControl::BreathingLight, AuroraLEDSpeed::Medium as u8,
				 AuroraLEDColour::Red, 0).await
	{
	    Status::CmdExecComplete => {
		info!("Fingerprint scanner LED set to blinking red, medium");

		return false;
	    },
	    Status::ErrorReceivePackage => {
		error!("Fingerprint scanner LED set - package receive");
	    },
	    stat => {
		error!("Unknown return code='{=u8:#04x}'", stat as u8);
	    }
	}

	return true;
    }

    pub async fn Wrapper_AuraSet_BlinkinRedFast(&mut self) -> bool {
	match self.AuraLedConfig(AuroraLEDControl::BreathingLight, AuroraLEDSpeed::Fast as u8,
				 AuroraLEDColour::Red, 0).await
	{
	    Status::CmdExecComplete => {
		info!("Fingerprint scanner LED set to blinking red, fast");

		return false;
	    },
	    Status::ErrorReceivePackage => {
		error!("Fingerprint scanner LED set - package receive");
	    },
	    stat => {
		error!("Unknown return code='{=u8:#04x}'", stat as u8);
	    }
	}

	return true;
    }

    pub async fn Wrapper_AuraSet_SteadyRed(&mut self) -> bool {
	match self.AuraLedConfig(AuroraLEDControl::AlwaysOn, 0, AuroraLEDColour::Red, 0).await {
	    Status::CmdExecComplete => {
		info!("Fingerprint scanner LED set to steady red");

		return false;
	    },
	    Status::ErrorReceivePackage => {
		error!("Fingerprint scanner LED set - package receive");
	    },
	    stat => {
		error!("Unknown return code='{=u8:#04x}'", stat as u8);
	    }
	}

	return true;
    }

    // -----

    pub async fn Wrapper_AuraSet_BlinkinBlueMedium(&mut self) -> bool {
	match self.AuraLedConfig(AuroraLEDControl::BreathingLight, AuroraLEDSpeed::Medium as u8,
				 AuroraLEDColour::Blue, 0).await
	{
	    Status::CmdExecComplete => {
		info!("Fingerprint scanner LED set to blinking blue, medium");

		return false;
	    },
	    Status::ErrorReceivePackage => {
		error!("Fingerprint scanner LED set - package receive");
	    },
	    stat => {
		error!("Unknown return code='{=u8:#04x}'", stat as u8);
	    }
	}

	return true;
    }

    pub async fn Wrapper_AuraSet_SteadyBlue(&mut self) -> bool {
	match self.AuraLedConfig(AuroraLEDControl::AlwaysOn, 0, AuroraLEDColour::Blue, 0).await {
	    Status::CmdExecComplete => {
		info!("Fingerprint scanner LED set to steady blue");

		return false;
	    },
	    Status::ErrorReceivePackage => {
		error!("Fingerprint scanner LED set - package receive");
	    },
	    stat => {
		error!("Unknown return code='{=u8:#04x}'", stat as u8);
	    }
	}

	return true;
    }

    pub async fn Wrapper_AuraSet_BlinkinPurpleMedium(&mut self) -> bool {
	match self.AuraLedConfig(AuroraLEDControl::BreathingLight, AuroraLEDSpeed::Medium as u8,
				 AuroraLEDColour::Purple, 0).await
	{
	    Status::CmdExecComplete => {
		info!("Fingerprint scanner LED set to blinking purple, medium");

		return false;
	    },
	    Status::ErrorReceivePackage => {
		error!("Fingerprint scanner LED set - package receive");
	    },
	    stat => {
		error!("Unknown return code='{=u8:#04x}'", stat as u8);
	    }
	}

	return true;
    }

    pub async fn Wrapper_AuraSet_SteadyPurpe(&mut self) -> bool {
	match self.AuraLedConfig(AuroraLEDControl::AlwaysOn, 0, AuroraLEDColour::Purple, 0).await {
	    Status::CmdExecComplete => {
		info!("Fingerprint scanner LED set to steady blue");

		return false;
	    },
	    Status::ErrorReceivePackage => {
		error!("Fingerprint scanner LED set - package receive");
	    },
	    stat => {
		error!("Unknown return code='{=u8:#04x}'", stat as u8);
	    }
	}

	return true;
    }

    // -----

    pub async fn Wrapper_AuraSet_Off(&mut self) -> bool {
	match self.AuraLedConfig(AuroraLEDControl::AlwaysOff, 0, AuroraLEDColour::Purple, 0).await {
	    Status::CmdExecComplete => {
		info!("Fingerprint scanner LED turned off");
		return false;
	    },
	    Status::ErrorReceivePackage => {
		error!("Fingerprint scanner LED set - package receive");
	    },
	    stat => {
		error!("Unknown return code='{=u8:#04x}'", stat as u8);
	    }
	}

	return true;
    }

    pub async fn Wrapper_Setup(&mut self) -> bool {
	match self.VfyPwd(self.password).await {
	    Status::CmdExecComplete => {
		info!("Fingerprint scanner password matches");
	    }
	    Status::ErrorReceivePackage => {
		error!("Package receive");

		self.Wrapper_AuraSet_BlinkinRedMedium().await;
		return true;
	    }
	    Status::ErrorPassword => {
		error!("Wrong password");

		self.Wrapper_AuraSet_BlinkinRedFast().await;
		return true;
	    }
	    stat => {
		error!("Unknown return code='{=u8:#04x}'", stat as u8);

		self.Wrapper_AuraSet_Off().await;
		return true;
	    }
	}

	match self.SoftRst().await {
	    Status::CmdExecComplete => {
		info!("Reset successful");
	    }
	    Status::ErrorReceivePackage => {
		error!("Package receive");

		self.Wrapper_AuraSet_BlinkinRedMedium().await;
		return true;
	    }
	    stat => {
		error!("Unknown return code='{=u8:#04x}'", stat as u8);

		self.Wrapper_AuraSet_Off().await;
		return true;
	    }
	}
	Timer::after_millis(500).await; // Give it half a second to come back up.

	match self.CheckSensor().await {
	    Status::CmdExecComplete => {
		info!("Sensor is normal");
	    }
	    Status::ErrorSensorAbnormal => {
		error!("Sensor is abnormal.");
		return true;
	    }
	    Status::ErrorReceivePackage => {
		error!("Package receive");

		self.Wrapper_AuraSet_BlinkinRedMedium().await;
		return true;
	    }
	    stat => {
		error!("Unknown return code='{=u8:#04x}'", stat as u8);

		self.Wrapper_AuraSet_Off().await;
		return true;
	    }
	}

	match self.ReadSysPara().await {
	    Status::CmdExecComplete => {
		info!("System parameters read");
	    }
	    Status::ErrorReceivePackage => {
		error!("Package receive");

		self.Wrapper_AuraSet_BlinkinRedMedium().await;
		return true;
	    }
	    stat => {
		error!("Unknown return code='{=u8:#04x}'", stat as u8);

		self.Wrapper_AuraSet_Off().await;
		return true;
	    }
	}

	return false;
    }

    pub async fn Wrapper_Get_Fingerprint(&mut self, store: u8) -> bool {
	// =====
	// Sometimes this scanner is a bit .. "sensitive". If you don't place your finger EXACTLY
	// right, it returns "no finger on sensor" (`ErrorNoFingerOnSensor`)!
	// So do this five times, with increasing delay, THEN fail.
	let mut attempt = 1;
	loop {
	    self.Wrapper_AuraSet_BlinkinBlueMedium().await;

	    info!("Place the finger on the scanner.");
	    if DISABLE_RW {
		Timer::after_millis(250).await; // Give it quarter of a sec for debug output to catch up.
	    } else {
		if self.wakeup.get_level() == Level::High {
		    self.wakeup.wait_for_low().await;
		} else {
		    self.wakeup.wait_for_high().await;
		}
	    }
	    debug!("  Finger detected");

	    // Scan the finger.
	    match self.GenImg().await {
		Status::CmdExecComplete => {
		    info!("Successfully got image.");

		    self.Wrapper_AuraSet_SteadyBlue().await;
		    break;
		}
		Status::ErrorReceivePackage => {
		    error!("Package receive");

		    self.Wrapper_AuraSet_BlinkinRedMedium().await;
		    return true;
		}
		Status::ErrorNoFingerOnSensor => {
		    if attempt >= 5 {
			error!("No finger on sensor");
			return true;
		    } else {
			info!("No finger on sensor - retrying in {:?} seconds. Attempt: {:?}/5", 5 * attempt, attempt);

			self.Wrapper_AuraSet_BlinkinRedSlow().await;
			Timer::after_secs(5 * attempt).await;

			attempt = attempt + 1;
		    }
		}
		Status::ErrorEnroleFinger => {
		    error!("Failed to enrole finger");
		}
		stat => {
		    error!("Unknown return code='{=u8:#04x}'", stat as u8);

		    self.Wrapper_AuraSet_Off().await;
		    return true;
		}
	    }
	}

	// Generate character file from the finger image.
	match self.Img2Tz(store).await {
	    Status::CmdExecComplete => {
		info!("Successfully generated character from fingerprint image");
	    }
	    Status::ErrorReceivePackage => {
		error!("Package receive");

		self.Wrapper_AuraSet_BlinkinRedMedium().await;
		return true;
	    }
	    Status::ErrorGenCharFileDistortedImage => {
		error!("Failed to generate character file due to distorted fingerprint image");

		self.Wrapper_AuraSet_BlinkinRedMedium().await;
		return true;
	    }
	    Status::ErrorGenCharFileSmallImage => {
		error!("Failed to generate character file due to too small image");

		self.Wrapper_AuraSet_BlinkinRedMedium().await;
		return true;
	    }
	    Status::ErrorMissingValidPrimaryImage => {
		error!("Failed to generate image because of lac of valid primary image");

		self.Wrapper_AuraSet_BlinkinRedMedium().await;
		return true;
	    }
	    stat => {
		error!("Unknown return code='{=u8:#04x}'", stat as u8);

		self.Wrapper_AuraSet_Off().await;
		return true;
	    }
	}

	return false;
    }

    pub async fn Wrapper_Enrole_Fingerprint(&mut self) -> bool {
	// =====
	// 1) Verify the password.
	if self.Wrapper_Setup().await {
	    error!("Can't setup scanner");

	    self.Wrapper_AuraSet_BlinkinRedMedium().await;
	    return true;
	} else {
	    info!("Setup complete.");

	    if self.Wrapper_AuraSet_SteadyBlue().await {
		error!("Can't set colour steady blue");

		return true;
	    } else {
		// =====
		// 2) Get the fingerprint - #1.
		self.Wrapper_AuraSet_BlinkinBlueMedium().await;

		if self.Wrapper_Get_Fingerprint(1).await {
		    error!("Couldn't scan the finger (first time)");

		    self.Wrapper_AuraSet_BlinkinRedMedium().await;
		    return true;
		} else {
		    info!("Scanned and saved the finger (first time)");

		    // =====
		    // 3) Get the fingerprint - #2.
		    self.Wrapper_AuraSet_BlinkinPurpleMedium().await;

		    if self.Wrapper_Get_Fingerprint(2).await {
			error!("Couldn't scan the finger (second time)");

			self.Wrapper_AuraSet_BlinkinRedMedium().await;
			return true;
		    } else {
			info!("Scanned and saved the finger (second time)");

			// =====
			// 4) Create a fingerprint model.
			match self.RegModel().await {
			    Status::CmdExecComplete => {
				info!("Fingerprint model generated");
			    }
			    Status::ErrorReceivePackage => {
				error!("Package receive");

				self.Wrapper_AuraSet_BlinkinRedMedium().await;
				return true;
			    }
			    Status::ErrorCombineCharFiles => {
				error!("Failed to combine character files");

				self.Wrapper_AuraSet_BlinkinRedMedium().await;
				return true;
			    }
			    stat => {
				error!("Unknown return code='{=u8:#04x}'", stat as u8);

				self.Wrapper_AuraSet_Off().await;
				return true;
			    }
			}

			// =====
			// 5) Store the fingerprint model in the flash.
			match self.Store(0x01, STORE).await {
			    Status::CmdExecComplete => {
				info!("Fingerprint model stored in the flash.");

				self.Wrapper_AuraSet_Off().await;
				return false;
			    }
			    Status::ErrorReceivePackage => {
				error!("Package receive");

				self.Wrapper_AuraSet_BlinkinRedMedium().await;
				return true;
			    }
			    Status::ErrorPageIdBeyondLibrary => {
				error!("Package ID beyond library");

				self.Wrapper_AuraSet_BlinkinRedMedium().await;
				return true;
			    }
			    Status::ErrorWriteFlash => {
				error!("Can't write flash");

				self.Wrapper_AuraSet_BlinkinRedMedium().await;
				return true;
			    }
			    stat => {
				error!("Unknown return code='{=u8:#04x}'", stat as u8);

				self.Wrapper_AuraSet_Off().await;
				return true;
			    }
			}
		    }
		}
	    }
	}
    }

    pub async fn Wrapper_Verify_Fingerprint(&mut self) -> bool {
	// =====
	// 1) Verify the password.
	if self.Wrapper_Setup().await {
	    error!("Can't setup scanner");

	    self.Wrapper_AuraSet_BlinkinRedMedium().await;
	    return true;
	} else {
	    info!("Setup complete.");

	    if self.Wrapper_AuraSet_SteadyBlue().await {
		error!("Can't set colour steady blue");

		return true;
	    } else {
		// =====
		// 2) Get the fingerprint - #1.
		if self.Wrapper_Get_Fingerprint(1).await {
		    error!("Couldn't scan the finger");

		    self.Wrapper_AuraSet_BlinkinRedMedium().await;
		    return true;
		} else {
		    info!("Scanned and saved the finger");

		    // =====
		    // 3) Search for the fingerprint
		    match self.Search(1, 0, 0xffff).await {
			Status::CmdExecComplete => {
			    info!("Fingerprint fount.");

			    self.Wrapper_AuraSet_Off().await;
			    return false;
			}
			Status::ErrorReceivePackage => {
			    error!("Package receive");

			    self.Wrapper_AuraSet_BlinkinRedMedium().await;
			    return true;
			}
			Status::ErrorNoMatchingFinger => {
			    error!("No matching finger");

			    self.Wrapper_AuraSet_BlinkinRedMedium().await;
			    return true;
			}
			stat => {
			    error!("Unknown return code='{=u8:#04x}'", stat as u8);

			    self.Wrapper_AuraSet_Off().await;
			    return true;
			}
		    }
		}
	    }
	}
    }
}
