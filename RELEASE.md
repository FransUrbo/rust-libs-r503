# Create a release

1. Update the `./Cargo.toml` file, bump the `version` entry.
2. Push the changes to GitHub.
3. Tag the commit with `REL_x_y_z`.
4. Push the tag to GitHub.
5. Run the workflow, save the artifacts locally.
6. Create a new release
   a. generate changelog between previous and current release tag.
   b. attach the artifacts from point #5.
