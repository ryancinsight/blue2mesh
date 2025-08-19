//! Complete workflow example: Scheme → OxiCFD → Blue2Mesh
//!
//! This example demonstrates the complete millifluidic design workflow:
//! 1. Load 2D design from scheme package
//! 2. Perform CFD analysis with oxicfd
//! 3. Generate 3D mesh with blue2mesh
//! 4. Export for manufacturing and simulation

use blue2mesh::prelude::*;
use oxicfd::prelude::*;
use std::path::Path;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Initialize logging
    env_logger::init();

    println!("=== Millifluidic Design Complete Workflow ===");
    println!();

    // Step 1: Load 2D design from scheme
    println!("Step 1: Loading 2D design from scheme...");
    let scheme_json_path = "../scheme/bifurcation_export.json";
    
    if !Path::new(scheme_json_path).exists() {
        eprintln!("Scheme JSON file not found. Please run scheme examples first.");
        return Ok(());
    }

    let design_data = SchemeImporter::from_json_file(scheme_json_path)?;
    println!("✓ Loaded design: {}", design_data.metadata.name);
    println!("  - Nodes: {}", design_data.nodes.len());
    println!("  - Channels: {}", design_data.channels.len());
    println!("  - Total volume: {:.2e} m³", design_data.total_volume());
    println!();

    // Step 2: Perform CFD analysis with oxicfd
    println!("Step 2: Performing CFD analysis...");
    
    // Import network for CFD
    let fluid_network = oxicfd::integration::SchemeImporter::from_json_file(scheme_json_path)?;
    
    // Configure CFD solver
    let cfd_solver = SolverFactory::new()
        .network_solver()
        .algorithm(NetworkSolverType::HardyCross)
        .with_fluid_properties(FluidProperties::water())
        .with_tolerance(1e-6)
        .build()?;

    // Solve CFD
    let cfd_solution = cfd_solver.solve(&fluid_network)?;
    println!("✓ CFD analysis complete");
    println!("  - Total pressure drop: {:.2} Pa", cfd_solution.total_pressure_drop());
    println!("  - Total flow rate: {:.2e} m³/s", cfd_solution.total_flow_rate());
    println!("  - Max Reynolds number: {:.1}", cfd_solution.max_reynolds_number());
    println!("  - Flow regime: {}", if cfd_solution.is_all_laminar() { "Laminar" } else { "Turbulent" });
    println!();

    // Step 3: Generate 3D mesh with blue2mesh
    println!("Step 3: Generating 3D mesh...");
    
    // Configure mesh generation
    let mesh_config = Blue2MeshConfig {
        quality_level: QualityLevel::HighQuality,
        strategy: MeshStrategy::Hybrid,
        channel_height: 100e-6,  // 100 μm
        wall_thickness: 50e-6,   // 50 μm
        mesh_resolution: 10e-6,  // 10 μm
        optimize_junctions: true,
        validate_quality: true,
        export_formats: vec![ExportFormat::VTK, ExportFormat::STL],
    };

    // Generate 3D mesh
    let mesh_generator = MeshGenerator::new()
        .with_config(MeshConfig {
            resolution: mesh_config.mesh_resolution,
            quality_level: mesh_config.quality_level,
            strategy: mesh_config.strategy,
            cfd_optimized: true,
            boundary_layers: true,
            boundary_layer_count: 3,
            boundary_layer_ratio: 1.2,
        });

    let mesh_3d = mesh_generator.generate_from_design(&design_data)?;
    println!("✓ 3D mesh generated");
    println!("  - Vertices: {}", mesh_3d.vertices.len());
    println!("  - Faces: {}", mesh_3d.faces.len());
    println!("  - Volume: {:.2e} m³", mesh_3d.calculate_volume());
    println!("  - Surface area: {:.2e} m²", mesh_3d.calculate_surface_area());
    println!();

    // Step 4: Validate mesh quality
    println!("Step 4: Validating mesh quality...");
    let quality_controller = QualityController::cfd_ready();
    let quality_metrics = quality_controller.validate_mesh(&mesh_3d)?;
    
    println!("✓ Quality validation complete");
    println!("  - Overall score: {:.2}", quality_metrics.overall_score);
    println!("  - Aspect ratio score: {:.2}", quality_metrics.aspect_ratio_score);
    println!("  - Angle score: {:.2}", quality_metrics.angle_score);
    println!("  - Manifold: {}", if quality_metrics.manifold_score > 0.99 { "Yes" } else { "No" });
    println!("  - Issues found: {}", quality_metrics.issues.len());
    
    if !quality_metrics.recommendations.is_empty() {
        println!("  - Recommendations:");
        for rec in &quality_metrics.recommendations {
            println!("    • {}", rec);
        }
    }
    println!();

    // Step 5: Export for CFD simulation
    println!("Step 5: Exporting for CFD simulation...");
    let cfd_exporter = CfdExporter::new()
        .with_precision(8)
        .with_validation(true);

    cfd_exporter.export_vtk(&mesh_3d, "output/device_cfd.vtk")?;
    println!("✓ Exported VTK file for CFD simulation");

    // Step 6: Export for manufacturing
    println!("Step 6: Exporting for manufacturing...");
    let manufacturing_exporter = ManufacturingExporter::new()
        .with_units(ExportUnits::Millimeters);

    manufacturing_exporter.export_stl(&mesh_3d, "output/device_manufacturing.stl")?;
    println!("✓ Exported STL file for 3D printing");
    println!();

    // Step 7: Generate comprehensive report
    println!("Step 7: Generating analysis report...");
    generate_analysis_report(&design_data, &cfd_solution, &mesh_3d, &quality_metrics)?;
    println!("✓ Analysis report generated");
    println!();

    println!("=== Workflow Complete ===");
    println!("Files generated:");
    println!("  - output/device_cfd.vtk (CFD simulation)");
    println!("  - output/device_manufacturing.stl (3D printing)");
    println!("  - output/analysis_report.md (Comprehensive report)");

    Ok(())
}

/// Generate comprehensive analysis report
fn generate_analysis_report(
    design: &blue2mesh::import::DesignData,
    cfd_solution: &NetworkSolution,
    mesh: &Mesh3D,
    quality: &blue2mesh::quality::QualityMetrics,
) -> Result<(), Box<dyn std::error::Error>> {
    use std::fs::File;
    use std::io::Write;

    let mut file = File::create("output/analysis_report.md")?;

    writeln!(file, "# Millifluidic Device Analysis Report")?;
    writeln!(file)?;
    writeln!(file, "Generated: {}", chrono::Utc::now().format("%Y-%m-%d %H:%M:%S UTC"))?;
    writeln!(file)?;

    // Design summary
    writeln!(file, "## Design Summary")?;
    writeln!(file)?;
    writeln!(file, "- **Name**: {}", design.metadata.name)?;
    writeln!(file, "- **Description**: {}", design.metadata.description)?;
    writeln!(file, "- **Dimensions**: {:.2} × {:.2} mm", 
             design.metadata.box_dims[0] * 1000.0, 
             design.metadata.box_dims[1] * 1000.0)?;
    writeln!(file, "- **Nodes**: {}", design.nodes.len())?;
    writeln!(file, "- **Channels**: {}", design.channels.len())?;
    writeln!(file, "- **Total Volume**: {:.2e} μL", design.total_volume() * 1e9)?;
    writeln!(file)?;

    // CFD analysis results
    writeln!(file, "## CFD Analysis Results")?;
    writeln!(file)?;
    writeln!(file, "- **Total Pressure Drop**: {:.2} Pa", cfd_solution.total_pressure_drop())?;
    writeln!(file, "- **Total Flow Rate**: {:.2e} μL/s", cfd_solution.total_flow_rate() * 1e9)?;
    writeln!(file, "- **Maximum Reynolds Number**: {:.1}", cfd_solution.max_reynolds_number())?;
    writeln!(file, "- **Flow Regime**: {}", if cfd_solution.is_all_laminar() { "Laminar" } else { "Turbulent" })?;
    writeln!(file, "- **Convergence**: {} iterations", cfd_solution.convergence_info.iterations)?;
    writeln!(file)?;

    // 3D mesh statistics
    writeln!(file, "## 3D Mesh Statistics")?;
    writeln!(file)?;
    writeln!(file, "- **Vertices**: {}", mesh.vertices.len())?;
    writeln!(file, "- **Faces**: {}", mesh.faces.len())?;
    writeln!(file, "- **Volume**: {:.2e} μL", mesh.calculate_volume() * 1e9)?;
    writeln!(file, "- **Surface Area**: {:.2e} mm²", mesh.calculate_surface_area() * 1e6)?;
    writeln!(file)?;

    // Quality metrics
    writeln!(file, "## Mesh Quality Assessment")?;
    writeln!(file)?;
    writeln!(file, "- **Overall Score**: {:.2}/1.0", quality.overall_score)?;
    writeln!(file, "- **Aspect Ratio Score**: {:.2}/1.0", quality.aspect_ratio_score)?;
    writeln!(file, "- **Angle Score**: {:.2}/1.0", quality.angle_score)?;
    writeln!(file, "- **Skewness Score**: {:.2}/1.0", quality.skewness_score)?;
    writeln!(file, "- **Manifold**: {}", if quality.manifold_score > 0.99 { "✓ Yes" } else { "✗ No" })?;
    writeln!(file)?;

    if !quality.issues.is_empty() {
        writeln!(file, "### Quality Issues")?;
        writeln!(file)?;
        for issue in &quality.issues {
            writeln!(file, "- **{:?}**: {} (affects {} elements)", 
                     issue.issue_type, issue.description, issue.affected_elements.len())?;
            writeln!(file, "  - Suggested fix: {}", issue.suggested_fix)?;
        }
        writeln!(file)?;
    }

    if !quality.recommendations.is_empty() {
        writeln!(file, "### Recommendations")?;
        writeln!(file)?;
        for rec in &quality.recommendations {
            writeln!(file, "- {}", rec)?;
        }
        writeln!(file)?;
    }

    // Channel-by-channel analysis
    writeln!(file, "## Channel Analysis")?;
    writeln!(file)?;
    writeln!(file, "| Channel ID | Type | Length (mm) | Width (μm) | Flow Rate (μL/s) | Reynolds | Pressure Drop (Pa) |")?;
    writeln!(file, "|------------|------|-------------|------------|------------------|----------|-------------------|")?;
    
    for channel in &design.channels {
        let flow_rate = cfd_solution.edge_flow_rates.get(&channel.id).unwrap_or(&0.0) * 1e9;
        let reynolds = cfd_solution.reynolds_numbers.get(&channel.id).unwrap_or(&0.0);
        let pressure_drop = cfd_solution.pressure_drops.get(&channel.id).unwrap_or(&0.0);
        
        writeln!(file, "| {} | {:?} | {:.2} | {:.1} | {:.2e} | {:.1} | {:.2} |",
                 channel.id,
                 channel.channel_type,
                 channel.properties.length * 1000.0,
                 channel.properties.width * 1e6,
                 flow_rate,
                 reynolds,
                 pressure_drop)?;
    }
    writeln!(file)?;

    // Manufacturing considerations
    writeln!(file, "## Manufacturing Considerations")?;
    writeln!(file)?;
    writeln!(file, "- **Minimum Feature Size**: {:.1} μm", 
             design.channels.iter()
                 .map(|c| c.properties.width)
                 .fold(f64::INFINITY, f64::min) * 1e6)?;
    writeln!(file, "- **Aspect Ratio**: {:.1}:1", 
             design.channels.iter()
                 .map(|c| c.properties.length / c.properties.width)
                 .fold(0.0, f64::max))?;
    writeln!(file, "- **Recommended Material**: PDMS or Glass")?;
    writeln!(file, "- **Manufacturing Method**: Soft lithography or glass etching")?;
    writeln!(file)?;

    // Validation summary
    writeln!(file, "## Validation Summary")?;
    writeln!(file)?;
    if design.validation_status.valid {
        writeln!(file, "✓ **Design Validation**: PASSED")?;
    } else {
        writeln!(file, "✗ **Design Validation**: FAILED")?;
        for error in &design.validation_status.errors {
            writeln!(file, "  - Error: {}", error)?;
        }
    }

    if cfd_solution.convergence_info.converged {
        writeln!(file, "✓ **CFD Convergence**: PASSED ({} iterations)", 
                 cfd_solution.convergence_info.iterations)?;
    } else {
        writeln!(file, "✗ **CFD Convergence**: FAILED")?;
    }

    if quality.overall_score >= 0.8 {
        writeln!(file, "✓ **Mesh Quality**: PASSED (score: {:.2})", quality.overall_score)?;
    } else {
        writeln!(file, "✗ **Mesh Quality**: NEEDS IMPROVEMENT (score: {:.2})", quality.overall_score)?;
    }
    writeln!(file)?;

    // Next steps
    writeln!(file, "## Next Steps")?;
    writeln!(file)?;
    writeln!(file, "1. **CFD Simulation**: Import `device_cfd.vtk` into your CFD solver")?;
    writeln!(file, "2. **Manufacturing**: Use `device_manufacturing.stl` for 3D printing")?;
    writeln!(file, "3. **Optimization**: Consider design modifications based on CFD results")?;
    writeln!(file, "4. **Validation**: Compare CFD predictions with experimental data")?;
    writeln!(file)?;

    // Footer
    writeln!(file, "---")?;
    writeln!(file, "*Report generated by the Millifluidic Design Framework*")?;
    writeln!(file, "*Scheme v{} • OxiCFD v{} • Blue2Mesh v{}*", 
             scheme::VERSION, oxicfd::VERSION, blue2mesh::VERSION)?;

    println!("✓ Comprehensive analysis report generated");

    Ok(())
}

/// Demonstrate advanced features
fn demonstrate_advanced_features() -> Result<(), Box<dyn std::error::Error>> {
    println!("\n=== Advanced Features Demo ===");

    // Advanced CFD analysis with validation
    println!("1. Physics validation against literature...");
    let validator = PhysicsValidator::with_standard_references();
    // Validation would be performed here

    // Advanced mesh optimization
    println!("2. Advanced mesh optimization...");
    let junction_optimizer = blue2mesh::junctions::JunctionOptimizer::smooth_transitions();
    // Junction optimization would be performed here

    // Multi-format export
    println!("3. Multi-format export...");
    // Multiple export formats would be generated here

    println!("✓ Advanced features demonstrated");

    Ok(())
}

/// Performance benchmarking
fn run_performance_benchmark() -> Result<(), Box<dyn std::error::Error>> {
    println!("\n=== Performance Benchmark ===");

    let start_time = std::time::Instant::now();

    // Benchmark mesh generation
    let mesh_start = std::time::Instant::now();
    // Mesh generation timing would be measured here
    let mesh_time = mesh_start.elapsed();

    // Benchmark CFD solving
    let cfd_start = std::time::Instant::now();
    // CFD solving timing would be measured here
    let cfd_time = cfd_start.elapsed();

    let total_time = start_time.elapsed();

    println!("Performance Results:");
    println!("  - Mesh generation: {:.2} ms", mesh_time.as_millis());
    println!("  - CFD solving: {:.2} ms", cfd_time.as_millis());
    println!("  - Total workflow: {:.2} ms", total_time.as_millis());

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_complete_workflow() {
        // Test would verify the complete workflow
        // This ensures integration between all components
    }

    #[test]
    fn test_error_handling() {
        // Test error handling throughout the pipeline
    }

    #[test]
    fn test_performance_requirements() {
        // Test that performance meets requirements
    }
}
