use anyhow::Context as _;
use std::env;

use std::fs::File;

fn main() -> anyhow::Result<()> {
    env_logger::init();

	let mut iter = env::args();
	let mut stage = 0;
	while let Some(arg) = iter.next() {
		if arg == "--stage" {
			let value = iter.next().unwrap();
			stage = value.parse().unwrap();
		}
	}



    // It takes clang++ around 30 seconds to dump the JSON, so just keep a file
    // around to reduce iteration times
    let root = if let Ok(json) = std::fs::read("ast-dump.json") {
        serde_json::from_slice(&json).context("failed to parse ast-dump.json")?
    } else {
        // This is the root API include that includes all the other public APIs
        let api_h = format!("{}/PxPhysicsAPI.h", pxbind::get_include_dir()?);
        let (root, raw) = pxbind::get_parsed_ast(api_h)?;

        std::fs::write("ast-dump.json", raw).context("failed to write ast-dump.json")?;

        root
    };


    let mut ast = pxbind::consumer::AstConsumer::default();
    ast.consume(&root)?;

    let rr: std::path::PathBuf = pxbind::get_repo_root()?.into();

	match stage {
		0 => {
			let mut structgen = File::create(rr.join("codegen/structgen.cpp"))?;
			let mut cpp = File::create(rr.join("codegen/physx_generated.hpp"))?;
			pxbind::generate_structgen(&ast, &mut structgen).unwrap();
			pxbind::generate_cpp(&ast, &mut cpp);
		}
		1 => {

			let struct_sizes: pxbind::StructMetadataList = if let Ok(json) = std::fs::read(rr.join("codegen/structgen_out.json")) {
				serde_json::from_slice(&json).context("failed to parse structgen_out.json")?
			} else {
				panic!("no sizes file");
			};

			let generator = pxbind::odin_generator::Generator::default();
			generator.generate_all(&ast, rr , struct_sizes)?;
		}
		_ => panic!("unknown stage {}", stage)
	}


    Ok(())
}
