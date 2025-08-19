//! Complete Scheme to STL Workflow Demonstration
//!
//! This example demonstrates the complete millifluidic design workflow:
//! 1. Create a simple channel design using scheme
//! 2. Export as JSON and PNG visualization
//! 3. Convert JSON to 3D STL using Blue2mesh
//! 4. Display both 2D and 3D outputs
//!
//! Run with: cargo run --example scheme_to_stl_demo -p blue2mesh

use scheme::{
    geometry::{generator::create_geometry, ChannelSystem},
    config::{GeometryConfig, ChannelTypeConfig},
    visualizations::schematic::plot_geometry,
};
use blue2mesh::quickstart::scheme_to_manufacturing;
use std::fs;
use std::path::Path;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("🔗 Complete Scheme to STL Workflow Demonstration");
    println!("================================================");
    println!();

    // Create output directories
    fs::create_dir_all("outputs/demo")?;
    fs::create_dir_all("outputs/demo/2d")?;
    fs::create_dir_all("outputs/demo/3d")?;

    // Step 1: Create a simple millifluidic design with scheme
    println!("1️⃣  Creating Millifluidic Design with Scheme");
    println!("   ==========================================");
    
    let channel_system = create_simple_channel_design()?;
    println!("   ✅ Created single channel design:");
    println!("      📐 Box dimensions: {:?} mm", channel_system.box_dims);
    println!("      🔗 Nodes: {}", channel_system.nodes.len());
    println!("      📏 Channels: {}", channel_system.channels.len());
    println!();

    // Step 2: Export scheme design as JSON and PNG
    println!("2️⃣  Exporting Scheme Design");
    println!("   =========================");
    
    let json_path = "outputs/demo/simple_channel.json";
    let png_path = "outputs/demo/2d/simple_channel.png";
    
    export_scheme_design(&channel_system, json_path, png_path)?;
    println!("   ✅ Exported scheme design:");
    println!("      📄 JSON: {}", json_path);
    println!("      🖼️  PNG:  {}", png_path);
    println!();

    // Step 3: Convert JSON to STL using Blue2mesh
    println!("3️⃣  Converting to 3D STL with Blue2mesh");
    println!("   ====================================");
    
    let stl_path = "outputs/demo/3d/simple_channel.stl";
    
    convert_to_stl(json_path, stl_path)?;
    println!("   ✅ Generated 3D STL mesh:");
    println!("      🏗️  STL: {}", stl_path);
    println!();

    // Step 4: Display results summary
    println!("4️⃣  Workflow Complete!");
    println!("   ==================");
    println!();
    println!("   📊 Results Summary:");
    println!("   ┌─────────────────────────────────────────────┐");
    println!("   │ 2D Design (Scheme)                          │");
    println!("   │ • JSON: {}                    │", json_path);
    println!("   │ • PNG:  {}                     │", png_path);
    println!("   │                                             │");
    println!("   │ 3D Mesh (Blue2mesh)                        │");
    println!("   │ • STL:  {}                     │", stl_path);
    println!("   └─────────────────────────────────────────────┘");
    println!();
    
    display_file_info(json_path, png_path, stl_path)?;
    
    println!("🎉 Complete millifluidic design workflow demonstrated!");
    println!("   You can now:");
    println!("   • View the 2D design: {}", png_path);
    println!("   • 3D print the device: {}", stl_path);
    println!("   • Use JSON for CFD analysis with OxiCFD");

    Ok(())
}

/// Create a simple single-channel millifluidic design
fn create_simple_channel_design() -> Result<ChannelSystem, Box<dyn std::error::Error>> {
    // Create a 96-well plate sized device (127.15mm x 85.75mm) with a single straight channel
    let system = create_geometry(
        (127.15, 85.75),  // Standard 96-well plate dimensions
        &[],              // No splits = single straight channel
        &GeometryConfig::default(),
        &ChannelTypeConfig::AllStraight,
    );

    println!("   📐 Design specifications:");
    println!("      • Device size: 127.15mm × 85.75mm (96-well plate standard)");
    println!("      • Device height: 10mm");
    println!("      • Channel type: Single straight channel");
    println!("      • Inlet at: ({:.1}, {:.1}) mm",
             system.nodes[0].point.0, system.nodes[0].point.1);
    println!("      • Outlet at: ({:.1}, {:.1}) mm",
             system.nodes[1].point.0, system.nodes[1].point.1);

    Ok(system)
}

/// Export scheme design as JSON and PNG
fn export_scheme_design(
    system: &ChannelSystem, 
    json_path: &str, 
    png_path: &str
) -> Result<(), Box<dyn std::error::Error>> {
    // Export to JSON for Blue2mesh
    let json = system.to_json()?;
    fs::write(json_path, &json)?;
    println!("   📄 JSON exported ({} bytes)", json.len());

    // Create PNG visualization
    plot_geometry(system, png_path)?;
    println!("   🖼️  PNG visualization created");

    Ok(())
}

/// Convert JSON to STL using Blue2mesh
fn convert_to_stl(json_path: &str, stl_path: &str) -> Result<(), Box<dyn std::error::Error>> {
    println!("   🔄 Processing with Blue2mesh...");
    
    // Use Blue2mesh to convert scheme JSON to manufacturing-ready STL
    scheme_to_manufacturing(json_path, stl_path)?;
    
    println!("   ✅ STL conversion completed");
    
    Ok(())
}

/// Display file information and sizes
fn display_file_info(
    json_path: &str, 
    png_path: &str, 
    stl_path: &str
) -> Result<(), Box<dyn std::error::Error>> {
    println!("   📁 File Information:");
    
    if Path::new(json_path).exists() {
        let size = fs::metadata(json_path)?.len();
        println!("   • JSON: {} bytes", size);
    }
    
    if Path::new(png_path).exists() {
        let size = fs::metadata(png_path)?.len();
        println!("   • PNG:  {} bytes", size);
    }
    
    if Path::new(stl_path).exists() {
        let size = fs::metadata(stl_path)?.len();
        println!("   • STL:  {} bytes", size);
    }
    
    Ok(())
}
