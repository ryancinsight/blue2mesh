//! Complete Scheme to STL workflow demonstration for multiple schematic types.
//!
//! This example generates and converts:
//! 1. Single straight channel
//! 2. Bifurcation
//! 3. Trifurcation
//! 4. Serpentine
//!
//! Run with: cargo run --example scheme_to_stl_demo -p blue2mesh

use blue2mesh::{ExtrusionConfig, quickstart::scheme_to_manufacturing_with_config};
use cfd_schematics::{
    config::{ChannelTypeConfig, GeometryConfig, SerpentineConfig},
    geometry::{generator::create_geometry, ChannelSystem, SplitType},
    visualizations::schematic::plot_geometry,
};
use std::fs;
use std::path::Path;

#[derive(Debug, Clone)]
struct DemoCase {
    name: &'static str,
    description: &'static str,
    box_dims_mm: (f64, f64),
    splits: Vec<SplitType>,
    channel_type_config: ChannelTypeConfig,
}

#[derive(Debug, Clone)]
struct OutputRecord {
    name: String,
    json_path: String,
    png_path: String,
    stl_path: String,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("🔗 Scheme to STL Workflow (Multi-Schematic)");
    println!("===========================================");
    println!();

    fs::create_dir_all("outputs/demo")?;
    fs::create_dir_all("outputs/demo/json")?;
    fs::create_dir_all("outputs/demo/2d")?;
    fs::create_dir_all("outputs/demo/3d")?;

    let geometry_config = GeometryConfig::default();
    let cases = vec![
        DemoCase {
            name: "single_channel",
            description: "Single straight channel (baseline)",
            box_dims_mm: (127.15, 85.75),
            splits: vec![],
            channel_type_config: ChannelTypeConfig::AllStraight,
        },
        DemoCase {
            name: "bifurcation",
            description: "Single bifurcation split",
            box_dims_mm: (200.0, 120.0),
            splits: vec![SplitType::Bifurcation],
            channel_type_config: ChannelTypeConfig::AllStraight,
        },
        DemoCase {
            name: "trifurcation",
            description: "Single trifurcation split",
            box_dims_mm: (240.0, 140.0),
            splits: vec![SplitType::Trifurcation],
            channel_type_config: ChannelTypeConfig::AllStraight,
        },
        DemoCase {
            name: "serpentine",
            description: "Bifurcation with serpentine channel paths",
            box_dims_mm: (260.0, 140.0),
            splits: vec![SplitType::Bifurcation],
            channel_type_config: ChannelTypeConfig::AllSerpentine(SerpentineConfig::default()),
        },
    ];

    let mut outputs = Vec::new();
    for case in &cases {
        let record = process_case(case, &geometry_config)?;
        outputs.push(record);
    }

    println!("📊 Output Summary");
    println!("=================");
    for output in &outputs {
        println!("• {}", output.name);
        display_file_info(&output.json_path, &output.png_path, &output.stl_path)?;
    }

    println!();
    println!("✅ Completed STL conversion for single, bifurcation, trifurcation, and serpentine schematics.");
    println!("   JSON outputs are also available for CFDrs-side preprocessing.");

    Ok(())
}

fn process_case(
    case: &DemoCase,
    geometry_config: &GeometryConfig,
) -> Result<OutputRecord, Box<dyn std::error::Error>> {
    println!("🧪 Processing: {}", case.name);
    println!("   {}", case.description);

    let channel_system = create_geometry(
        case.box_dims_mm,
        &case.splits,
        geometry_config,
        &case.channel_type_config,
    );
    println!(
        "   📐 Box: {:?} mm | Nodes: {} | Channels: {}",
        channel_system.box_dims,
        channel_system.nodes.len(),
        channel_system.channels.len()
    );

    let json_path = format!("outputs/demo/json/{}.json", case.name);
    let png_path = format!("outputs/demo/2d/{}.png", case.name);
    let stl_path = format!("outputs/demo/3d/{}.stl", case.name);

    export_scheme_design(&channel_system, &json_path, &png_path)?;
    convert_to_stl(&json_path, &stl_path)?;
    println!("   ✅ Done: {}", stl_path);
    println!();

    Ok(OutputRecord {
        name: case.name.to_string(),
        json_path,
        png_path,
        stl_path,
    })
}

/// Export scheme design as JSON and PNG.
fn export_scheme_design(
    system: &ChannelSystem,
    json_path: &str,
    png_path: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    let json = system.to_json()?;
    fs::write(json_path, &json)?;
    plot_geometry(system, png_path)?;
    Ok(())
}

/// Convert JSON to STL using Blue2mesh.
fn convert_to_stl(json_path: &str, stl_path: &str) -> Result<(), Box<dyn std::error::Error>> {
    let diameter_scale = std::env::var("B2M_CHANNEL_SCALE")
        .ok()
        .and_then(|v| v.parse::<f64>().ok())
        .unwrap_or(3.0);
    let max_path_segments = std::env::var("B2M_MAX_PATH_SEGMENTS")
        .ok()
        .and_then(|v| v.parse::<usize>().ok())
        .unwrap_or(8);
    let channel_segments = std::env::var("B2M_CHANNEL_SEGMENTS")
        .ok()
        .and_then(|v| v.parse::<usize>().ok())
        .unwrap_or(6);

    let extrusion_config = ExtrusionConfig::new()
        .with_csg_operations(true)
        .with_csg_channel_diameter_scale(diameter_scale)
        .with_csg_max_path_segments(max_path_segments)
        .with_csg_channel_segments(channel_segments);

    scheme_to_manufacturing_with_config(json_path, stl_path, extrusion_config)?;
    Ok(())
}

/// Display file information and sizes.
fn display_file_info(
    json_path: &str,
    png_path: &str,
    stl_path: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    if Path::new(json_path).exists() {
        let size = fs::metadata(json_path)?.len();
        println!("  - JSON: {} bytes ({})", size, json_path);
    }

    if Path::new(png_path).exists() {
        let size = fs::metadata(png_path)?.len();
        println!("  - PNG:  {} bytes ({})", size, png_path);
    }

    if Path::new(stl_path).exists() {
        let size = fs::metadata(stl_path)?.len();
        println!("  - STL:  {} bytes ({})", size, stl_path);
    }

    Ok(())
}
