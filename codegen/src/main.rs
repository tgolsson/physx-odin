use anyhow::Context as _;

const OUT_DIR: &'static str = env!("OUT_DIR");


#[cfg(target_os = "windows")]
mod files {
	pub const PHYSX_API_NAME: &'static str = "physx_api.lib";

	pub const PHYSX_NAME: &'static str = "physx.lib";


	#[cfg(debug_assertions)]
	pub const PHYSX_API_TARGET_NAME: &'static str = "physx_api.lib";
	#[cfg(debug_assertions)]
	pub const PHYSX_TARGET_NAME: &'static str = "physx.lib";

	#[cfg(not(debug_assertions))]
	pub const PHYSX_API_TARGET_NAME: &'static str = "physx_api_release.lib";
	#[cfg(not(debug_assertions))]
	pub const PHYSX_TARGET_NAME: &'static str = "physx_release.lib";
}

#[cfg(target_os = "linux")]
mod files {
	pub const LIBPHYSX_API_NAME: &'static str = "libphysx_api.so";

	pub const LIBPHYSX_NAME: &'static str = "libphysx.so";


	#[cfg(debug_assertions)]
	pub const LIBPHYSX_API_TARGET_NAME: &'static str = "libphysx_api.so";
	#[cfg(debug_assertions)]
	pub const LIBPHYSX_TARGET_NAME: &'static str = "libphysx.so";

	#[cfg(not(debug_assertions))]
	pub const LIBPHYSX_API_TARGET_NAME: &'static str = "libphysx_api_release.so";
	#[cfg(not(debug_assertions))]
	pub const LIBPHYSX_TARGET_NAME: &'static str = "libphysx_release.so";
}

use files::*;

pub fn get_repo_root() -> anyhow::Result<String> {
    let mut git = std::process::Command::new("git");
    git.args(["rev-parse", "--show-toplevel"]);
    git.stdout(std::process::Stdio::piped());
    let captured = git
        .output()
        .context("failed to run git to find repo root")?;

    let mut rr = String::from_utf8(captured.stdout).context("git output was non-utf8")?;
    // Removing trailing newline
    rr.pop();
    Ok(rr)
}



use std::path::PathBuf;

fn main() -> anyhow::Result<()>{
	let rr: PathBuf = get_repo_root()?.into();
	let out_dir: PathBuf = OUT_DIR.into();

	std::fs::copy(out_dir.join(LIBPHYSX_API_NAME), rr.join(LIBPHYSX_API_TARGET_NAME))?;
	std::fs::copy(out_dir.join(LIBPHYSX_NAME), rr.join(LIBPHYSX_TARGET_NAME))?;

	Ok(())
}
