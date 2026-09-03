//! This build script copies the `memory.x` file from the crate root into
//! a directory where the linker can always find it at build time.
//! For many projects this is optional, as the linker always searches the
//! project root directory -- wherever `Cargo.toml` is. However, if you
//! are using a workspace or have a more complicated build setup, this
//! build script becomes required. Additionally, by requesting that
//! Cargo re-run the build script whenever `memory.x` is changed,
//! updating `memory.x` ensures a rebuild of the application with the
//! new memory settings.

use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    zip_leveler_phyphox();
    // Put `memory.x` in our output directory and ensure it's
    // on the linker search path.
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();
    println!("cargo::rustc-link-search={}", out.display());

    // By default, Cargo will re-run a build script whenever
    // any file in the project changes. By specifying `memory.x`
    // here, we ensure the build script is only re-run when
    // `memory.x` is changed.
    println!("cargo::rerun-if-changed=memory.x");

    println!("cargo::rustc-link-arg-bins=--nmagic");
    println!("cargo::rustc-link-arg-bins=-Tlink.x");
    println!("cargo::rustc-link-arg-bins=-Tdefmt.x");
}

use std::io::Read;
use zip::ZipWriter;
use zip::write::SimpleFileOptions;

fn zip_leveler_phyphox() {
    // Re-run this build script if leveler.phyphox changes
    println!("cargo:rerun-if-changed=leveler.phyphox");

    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());
    let zip_path = out_dir.join("leveler.zip");

    // Read the XML file
    let mut xml_file = File::open("leveler.phyphox").expect("Failed to open leveler.phyphox");
    let mut xml_data = Vec::new();
    xml_file.read_to_end(&mut xml_data).unwrap();

    // Create the ZIP archive
    let zip_file = File::create(&zip_path).expect("Failed to create ZIP output");
    let mut zip = ZipWriter::new(zip_file);

    let options = SimpleFileOptions::default().compression_method(zip::CompressionMethod::Deflated);

    zip.start_file("leveler.phyphox", options).unwrap();
    zip.write_all(&xml_data).unwrap();
    zip.finish().unwrap();
}
