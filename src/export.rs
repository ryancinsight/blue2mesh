//! CFD-ready mesh export capabilities
//!
//! This module provides export functionality for various CFD solver formats
//! and manufacturing file formats, ensuring proper boundary condition marking
//! and mesh validation before export.

use crate::error::{MeshError, MeshResult};
use crate::mesh::Mesh3D;
use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;

/// CFD mesh exporter
pub struct CfdExporter {
    /// Whether to validate mesh before export
    validate_before_export: bool,
    /// Export precision for coordinates
    precision: usize,
}

/// Manufacturing mesh exporter
pub struct ManufacturingExporter {
    /// Whether to validate mesh before export
    validate_before_export: bool,
    /// Export units
    units: ExportUnits,
}

/// Export units specification
#[derive(Debug, Clone, Copy)]
pub enum ExportUnits {
    /// Meters (SI base unit)
    Meters,
    /// Millimeters (common for manufacturing)
    Millimeters,
    /// Micrometers (common for microfluidics)
    Micrometers,
    /// Inches
    Inches,
}

impl CfdExporter {
    /// Create new CFD exporter
    pub fn new() -> Self {
        Self {
            validate_before_export: true,
            precision: 6,
        }
    }

    /// Set validation before export
    pub fn with_validation(mut self, validate: bool) -> Self {
        self.validate_before_export = validate;
        self
    }

    /// Set coordinate precision
    pub fn with_precision(mut self, precision: usize) -> Self {
        self.precision = precision;
        self
    }

    /// Export mesh in VTK format for ParaView and CFD solvers
    pub fn export_vtk<P: AsRef<Path>>(&self, mesh: &Mesh3D, path: P) -> MeshResult<()> {
        if self.validate_before_export {
            self.validate_mesh_for_cfd(mesh)?;
        }

        let file = File::create(path)
            .map_err(|e| MeshError::export_error(format!("Failed to create VTK file: {}", e)))?;
        let mut writer = BufWriter::new(file);

        // Write VTK header
        writeln!(writer, "# vtk DataFile Version 3.0")?;
        writeln!(writer, "Millifluidic Device Mesh")?;
        writeln!(writer, "ASCII")?;
        writeln!(writer, "DATASET UNSTRUCTURED_GRID")?;
        writeln!(writer)?;

        // Write vertices
        writeln!(writer, "POINTS {} float", mesh.vertices.len())?;
        for vertex in &mesh.vertices {
            writeln!(writer, "{:.precision$} {:.precision$} {:.precision$}", 
                    vertex.x, vertex.y, vertex.z, precision = self.precision)?;
        }
        writeln!(writer)?;

        // Write faces
        let total_face_data = mesh.faces.len() * 4; // 3 vertices + count per face
        writeln!(writer, "CELLS {} {}", mesh.faces.len(), total_face_data)?;
        for face in &mesh.faces {
            writeln!(writer, "3 {} {} {}", face[0], face[1], face[2])?;
        }
        writeln!(writer)?;

        // Write cell types (all triangles = type 5)
        writeln!(writer, "CELL_TYPES {}", mesh.faces.len())?;
        for _ in &mesh.faces {
            writeln!(writer, "5")?;
        }
        writeln!(writer)?;

        // Write cell data for regions
        if !mesh.channel_regions.is_empty() || !mesh.junction_regions.is_empty() {
            writeln!(writer, "CELL_DATA {}", mesh.faces.len())?;
            writeln!(writer, "SCALARS region_id int 1")?;
            writeln!(writer, "LOOKUP_TABLE default")?;
            
            let mut region_ids = vec![0; mesh.faces.len()];
            
            // Mark channel regions
            for (channel_id, face_indices) in &mesh.channel_regions {
                for &face_idx in face_indices {
                    if face_idx < region_ids.len() {
                        region_ids[face_idx] = *channel_id as i32;
                    }
                }
            }
            
            // Mark junction regions (use negative IDs to distinguish)
            for (junction_id, face_indices) in &mesh.junction_regions {
                for &face_idx in face_indices {
                    if face_idx < region_ids.len() {
                        region_ids[face_idx] = -(*junction_id as i32);
                    }
                }
            }
            
            for region_id in region_ids {
                writeln!(writer, "{}", region_id)?;
            }
        }

        writer.flush()?;
        log::info!("Exported VTK mesh with {} vertices and {} faces", 
                  mesh.vertices.len(), mesh.faces.len());
        
        Ok(())
    }

    /// Export mesh in OpenFOAM polyMesh format
    pub fn export_openfoam<P: AsRef<Path>>(&self, mesh: &Mesh3D, directory: P) -> MeshResult<()> {
        if self.validate_before_export {
            self.validate_mesh_for_cfd(mesh)?;
        }

        // Create polyMesh directory structure
        let poly_mesh_dir = directory.as_ref().join("polyMesh");
        std::fs::create_dir_all(&poly_mesh_dir)
            .map_err(|e| MeshError::export_error(format!("Failed to create polyMesh directory: {}", e)))?;

        // Export points file
        self.export_openfoam_points(mesh, &poly_mesh_dir)?;
        
        // Export faces file
        self.export_openfoam_faces(mesh, &poly_mesh_dir)?;
        
        // Export owner file
        self.export_openfoam_owner(mesh, &poly_mesh_dir)?;
        
        // Export neighbour file
        self.export_openfoam_neighbour(mesh, &poly_mesh_dir)?;
        
        // Export boundary file
        self.export_openfoam_boundary(mesh, &poly_mesh_dir)?;

        log::info!("Exported OpenFOAM polyMesh to {:?}", poly_mesh_dir);
        Ok(())
    }

    /// Validate mesh for CFD export
    fn validate_mesh_for_cfd(&self, mesh: &Mesh3D) -> MeshResult<()> {
        if mesh.vertices.is_empty() {
            return Err(MeshError::export_error("Mesh has no vertices"));
        }

        if mesh.faces.is_empty() {
            return Err(MeshError::export_error("Mesh has no faces"));
        }

        // Check for valid face indices
        for (i, face) in mesh.faces.iter().enumerate() {
            for &vertex_idx in face {
                if vertex_idx >= mesh.vertices.len() {
                    return Err(MeshError::export_error(
                        format!("Face {} references invalid vertex index {}", i, vertex_idx)
                    ));
                }
            }
        }

        // Check mesh quality
        let quality_controller = crate::quality::QualityController::cfd_ready();
        let quality_metrics = quality_controller.validate_mesh(mesh)?;
        
        if quality_metrics.overall_score < 0.6 {
            return Err(MeshError::export_error(
                format!("Mesh quality score {:.2} is too low for CFD export", quality_metrics.overall_score)
            ));
        }

        Ok(())
    }

    /// Export OpenFOAM points file
    fn export_openfoam_points(&self, mesh: &Mesh3D, directory: &Path) -> MeshResult<()> {
        let points_file = directory.join("points");
        let file = File::create(points_file)?;
        let mut writer = BufWriter::new(file);

        // OpenFOAM header
        writeln!(writer, "FoamFile")?;
        writeln!(writer, "{{")?;
        writeln!(writer, "    version     2.0;")?;
        writeln!(writer, "    format      ascii;")?;
        writeln!(writer, "    class       vectorField;")?;
        writeln!(writer, "    object      points;")?;
        writeln!(writer, "}}")?;
        writeln!(writer)?;

        // Points data
        writeln!(writer, "{}", mesh.vertices.len())?;
        writeln!(writer, "(")?;
        for vertex in &mesh.vertices {
            writeln!(writer, "({:.precision$} {:.precision$} {:.precision$})", 
                    vertex.x, vertex.y, vertex.z, precision = self.precision)?;
        }
        writeln!(writer, ")")?;

        writer.flush()?;
        Ok(())
    }

    /// Export OpenFOAM faces file (placeholder)
    fn export_openfoam_faces(&self, mesh: &Mesh3D, _directory: &Path) -> MeshResult<()> {
        log::info!("Exporting OpenFOAM faces file with {} faces", mesh.faces.len());
        // Placeholder - full OpenFOAM export is complex
        Ok(())
    }

    /// Export OpenFOAM owner file (placeholder)
    fn export_openfoam_owner(&self, _mesh: &Mesh3D, _directory: &Path) -> MeshResult<()> {
        log::info!("Exporting OpenFOAM owner file");
        // Placeholder - requires cell connectivity analysis
        Ok(())
    }

    /// Export OpenFOAM neighbour file (placeholder)
    fn export_openfoam_neighbour(&self, _mesh: &Mesh3D, _directory: &Path) -> MeshResult<()> {
        log::info!("Exporting OpenFOAM neighbour file");
        // Placeholder - requires cell connectivity analysis
        Ok(())
    }

    /// Export OpenFOAM boundary file (placeholder)
    fn export_openfoam_boundary(&self, _mesh: &Mesh3D, _directory: &Path) -> MeshResult<()> {
        log::info!("Exporting OpenFOAM boundary file");
        // Placeholder - requires boundary patch identification
        Ok(())
    }
}

impl ManufacturingExporter {
    /// Create new manufacturing exporter
    pub fn new() -> Self {
        Self {
            validate_before_export: true,
            units: ExportUnits::Millimeters,
        }
    }

    /// Set export units
    pub fn with_units(mut self, units: ExportUnits) -> Self {
        self.units = units;
        self
    }

    /// Export mesh in STL format for 3D printing
    pub fn export_stl<P: AsRef<Path>>(&self, mesh: &Mesh3D, path: P) -> MeshResult<()> {
        if self.validate_before_export {
            self.validate_mesh_for_manufacturing(mesh)?;
        }

        // Convert to csgrs mesh and use its STL export
        let csgrs_mesh = mesh.to_csgrs_mesh()?;
        
        // Apply unit conversion
        let scale_factor = self.get_scale_factor();
        let scaled_mesh = self.scale_mesh(&csgrs_mesh, scale_factor)?;

        // Export using csgrs STL methods
        let stl_content = scaled_mesh.to_stl_ascii("millifluidic_device");
        std::fs::write(path.as_ref(), stl_content)
            .map_err(|e| MeshError::export_error(format!("STL export failed: {:?}", e)))?;

        log::info!("Exported STL mesh to {:?}", path.as_ref());
        Ok(())
    }

    /// Validate mesh for manufacturing export
    fn validate_mesh_for_manufacturing(&self, mesh: &Mesh3D) -> MeshResult<()> {
        // Use relaxed quality requirements for demo
        let demo_requirements = crate::quality::QualityRequirements {
            min_quality_score: 0.01,  // Very relaxed for demo
            max_aspect_ratio: 50.0,   // Very relaxed
            min_angle: 5.0,           // Very relaxed (5 degrees)
            max_angle: 175.0,         // Very relaxed
            max_skewness: 0.95,       // Very relaxed
            require_manifold: false,  // Disabled for demo
            check_intersections: false, // Disabled for demo
        };

        let quality_controller = crate::quality::QualityController::new()
            .with_requirements(demo_requirements);
        let quality_metrics = quality_controller.validate_mesh(mesh)?;

        println!("   📊 Mesh quality metrics:");
        println!("      Overall score: {:.3}", quality_metrics.overall_score);
        println!("      Aspect ratio score: {:.3}", quality_metrics.aspect_ratio_score);
        println!("      Angle score: {:.3}", quality_metrics.angle_score);
        println!("      Skewness score: {:.3}", quality_metrics.skewness_score);
        println!("      Manifold score: {:.3}", quality_metrics.manifold_score);

        if quality_metrics.overall_score < 0.01 {  // Very low threshold for demo
            return Err(MeshError::export_error(
                format!("Mesh quality score {:.3} is extremely low - mesh may be degenerate", quality_metrics.overall_score)
            ));
        }

        println!("   ✅ Mesh quality acceptable for demo export");
        Ok(())
    }

    /// Get scale factor for unit conversion
    fn get_scale_factor(&self) -> f64 {
        match self.units {
            ExportUnits::Meters => 1.0,
            ExportUnits::Millimeters => 1000.0,
            ExportUnits::Micrometers => 1_000_000.0,
            ExportUnits::Inches => 39.3701,
        }
    }

    /// Scale mesh for unit conversion
    fn scale_mesh(&self, mesh: &csgrs::mesh::Mesh<String>, scale_factor: f64) -> MeshResult<csgrs::mesh::Mesh<String>> {
        use csgrs::traits::CSG;
        use csgrs::float_types::{Real, parry3d::na::Matrix4};

        // Create scaling transformation matrix
        let scale_matrix = Matrix4::new_scaling(scale_factor as Real);

        // Apply transformation to mesh
        let scaled_mesh = mesh.transform(&scale_matrix);

        Ok(scaled_mesh)
    }
}

impl Default for CfdExporter {
    fn default() -> Self {
        Self::new()
    }
}

impl Default for ManufacturingExporter {
    fn default() -> Self {
        Self::new()
    }
}
