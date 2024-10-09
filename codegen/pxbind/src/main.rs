use anyhow::Context as _;
use pxbind::type_db::TypeDB;
use std::env;

use std::fs::File;

fn dump_clang_ast() -> anyhow::Result<pxbind::Node> {
    println!("  Starting AST dump... ");
    if let Ok(json) = std::fs::read("ast-dump.json") {
        println!("    AST loaded from file.");
        Ok(serde_json::from_slice(&json).context("failed to parse ast-dump.json")?)
    } else {
        println!("    generating with clang... ");
        // This is the root API include that includes all the other public APIs
        let api_h = format!("{}/PxPhysicsAPI.h", pxbind::get_include_dir()?);
        let (root, raw) = pxbind::get_parsed_ast(api_h)?;

        std::fs::write("ast-dump.json", raw).context("failed to write ast-dump.json")?;
        println!("    wrote cache to file.");
        Ok(root)
    }
}

fn dump_type_db() -> anyhow::Result<TypeDB> {
    println!("Starting Type dump... ");
    if let Ok(json) = std::fs::read("type-db.json") {
        println!("  loaded from file.");
        Ok(serde_json::from_slice(&json).context("failed to parse type-db.json")?)
    } else {
        let root = dump_clang_ast()?;
        println!("  converting AST to type dump ");
        let mut ast = pxbind::consumer::AstConsumer::default();
        ast.consume(&root)?;
        let type_db = TypeDB::from(&ast);
        std::fs::write("type-db.json", serde_json::to_string_pretty(&type_db)?)
            .context("failed to write type-db.json")?;
        println!("  wrote cache to file.");
        Ok(type_db)
    }
}

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

    let type_db = dump_type_db()?;
    let rr: std::path::PathBuf = pxbind::get_repo_root()?.into();

    match stage {
        0 => {
            let mut structgen = File::create(rr.join("codegen/structgen.cpp"))?;
            let mut cpp = File::create(rr.join("codegen/physx_generated.hpp"))?;
            pxbind::generate_structgen(&type_db, &mut structgen).unwrap();
            pxbind::generate_cpp(&type_db, &mut cpp);
        }
        1 => {
            let struct_sizes: pxbind::StructMetadataList =
                if let Ok(json) = std::fs::read(rr.join("codegen/structgen_out.json")) {
                    serde_json::from_slice(&json).context("failed to parse structgen_out.json")?
                } else {
                    panic!("no sizes file");
                };

            let generator = pxbind::odin_generator::Generator::default();
            generator.generate_all(&type_db, rr, struct_sizes)?;
        }
        _ => panic!("unknown stage {}", stage),
    }

    Ok(())
}
