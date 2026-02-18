//! `Blue2Mesh` - 3D Mesh Generation for Millifluidic Devices
//!
//! A specialized library for converting 2D millifluidic designs from the scheme
//! package into high-quality 3D meshes suitable for CFD simulation and manufacturing.
//! Emphasizes mesh quality, CFD readiness, and seamless integration with oxicfd.
//!
//! # Architecture
//!
//! `Blue2Mesh` follows SOLID and CUPID design principles with a modular architecture:
//!
//! - **import**: Scheme JSON parsing and validation
//! - **extrusion**: 2D to 3D extrusion algorithms with channel-specific handling
//! - **mesh**: High-quality mesh generation with adaptive refinement
//! - **channels**: Specialized mesh generators for different channel types
//! - **junctions**: Optimized junction mesh generation for smooth flow transitions
//! - **quality**: Mesh quality validation and improvement algorithms
//! - **export**: CFD-ready mesh export in multiple formats
//!
//! # Design Patterns
//!
//! - **Strategy Pattern**: Multiple extrusion algorithms for different channel types
//! - **Factory Pattern**: Mesh generator creation and configuration
//! - **Builder Pattern**: Complex mesh configuration and assembly
//! - **Template Method**: Standardized mesh generation workflow
//!
//! # Examples
//!
//! ## Basic 3D Extrusion
//!
//! ```rust,no_run
//! # fn main() -> Result<(), Box<dyn std::error::Error>> {
//! use blue2mesh::{
//!     import::SchemeImporter,
//!     extrusion::{ExtrusionConfig, ExtrusionEngine},
//!     export::CfdExporter,
//! };
//!
//! // Import 2D design from scheme
//! let design = SchemeImporter::from_json_file("design.json")?;
//!
//! // Configure 3D extrusion
//! let config = ExtrusionConfig::new()
//!     .with_height(100e-6)  // 100 μm channel height
//!     .with_wall_thickness(50e-6)  // 50 μm wall thickness
//!     .with_mesh_resolution(10e-6);  // 10 μm mesh resolution
//!
//! // Generate 3D mesh
//! let mesh = ExtrusionEngine::new(config)
//!     .extrude_design(&design)?;
//!
//! // Export for CFD
//! CfdExporter::new()
//!     .export_vtk(&mesh, "device.vtk")?;
//! # Ok(())
//! # }
//! ```
//!
//! ## Advanced Mesh Generation
//!
//! ```rust,no_run
//! # fn main() -> Result<(), Box<dyn std::error::Error>> {
//! use blue2mesh::{
//!     mesh::MeshGenerator,
//!     QualityLevel,
//! };
//!
//! // Generate high-quality mesh
//! let mesh = MeshGenerator::new()
//!     .with_quality_level(QualityLevel::HighQuality)
//!     .with_resolution(5e-6)  // 5 μm resolution
//!     .generate_from_scheme("design.json")?;
//! # Ok(())
//! # }
//! ```

pub mod error;
pub mod import;
pub mod extrusion;
pub mod mesh;
pub mod channels;
pub mod junctions;
pub mod quality;
pub mod export;

#[cfg(feature = "visualization")]
pub mod visualization;

// Re-export commonly used types
pub use error::{MeshError, MeshResult};
pub use import::{SchemeImporter, DesignData};
pub use extrusion::{ExtrusionConfig, ExtrusionEngine, ExtrusionResult, CsgBuildConfig};
pub use mesh::{MeshGenerator, Mesh3D, MeshConfig};
pub use quality::{QualityController, QualityMetrics};
pub use export::{CfdExporter, ManufacturingExporter};

/// Library version information
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// Default mesh generation parameters
pub mod defaults {
    /// Default channel height (m) - 100 μm
    pub const DEFAULT_CHANNEL_HEIGHT: f64 = 100e-6;
    
    /// Default wall thickness (m) - 50 μm
    pub const DEFAULT_WALL_THICKNESS: f64 = 50e-6;
    
    /// Default mesh resolution (m) - 10 μm
    pub const DEFAULT_MESH_RESOLUTION: f64 = 10e-6;
    
    /// Default aspect ratio limit for mesh quality
    pub const DEFAULT_ASPECT_RATIO_LIMIT: f64 = 10.0;
    
    /// Default minimum mesh angle (degrees)
    pub const DEFAULT_MIN_ANGLE: f64 = 20.0;
    
    /// Default maximum mesh angle (degrees)
    pub const DEFAULT_MAX_ANGLE: f64 = 140.0;
    
    /// Default junction smoothing radius (m)
    pub const DEFAULT_JUNCTION_RADIUS: f64 = 25e-6;
}

/// Mesh generation quality levels
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QualityLevel {
    /// Fast generation with basic quality
    Fast,
    /// Balanced quality and performance
    Balanced,
    /// High quality for accurate CFD
    HighQuality,
    /// Maximum quality for research applications
    Research,
}

/// Mesh generation strategy
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MeshStrategy {
    /// Structured mesh generation
    Structured,
    /// Unstructured mesh generation
    Unstructured,
    /// Hybrid structured/unstructured
    Hybrid,
    /// Adaptive mesh refinement
    Adaptive,
}

/// Export format specification
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExportFormat {
    /// VTK format for `ParaView` and CFD solvers
    VTK,
    /// `OpenFOAM` polyMesh format
    OpenFOAM,
    /// CGNS format for advanced CFD
    CGNS,
    /// STL format for manufacturing
    STL,
    /// OBJ format for visualization
    OBJ,
    /// PLY format for point clouds
    PLY,
}

/// Prelude module for convenient imports
pub mod prelude {
    pub use crate::{
        MeshError, MeshResult,
        SchemeImporter, DesignData,
        ExtrusionConfig, ExtrusionEngine, ExtrusionResult, CsgBuildConfig,
        MeshGenerator, Mesh3D, MeshConfig,
        QualityController, QualityMetrics,
        CfdExporter, ManufacturingExporter,
        QualityLevel, MeshStrategy, ExportFormat,
    };
    
    pub use crate::defaults::*;
}

/// Configuration for the entire blue2mesh pipeline
#[derive(Debug, Clone)]
pub struct Blue2MeshConfig {
    /// Mesh generation quality level
    pub quality_level: QualityLevel,
    /// Mesh generation strategy
    pub strategy: MeshStrategy,
    /// Channel height (m)
    pub channel_height: f64,
    /// Wall thickness (m)
    pub wall_thickness: f64,
    /// Mesh resolution (m)
    pub mesh_resolution: f64,
    /// Whether to optimize junctions
    pub optimize_junctions: bool,
    /// Whether to validate mesh quality
    pub validate_quality: bool,
    /// Export formats to generate
    pub export_formats: Vec<ExportFormat>,
}

impl Default for Blue2MeshConfig {
    fn default() -> Self {
        Self {
            quality_level: QualityLevel::Balanced,
            strategy: MeshStrategy::Hybrid,
            channel_height: defaults::DEFAULT_CHANNEL_HEIGHT,
            wall_thickness: defaults::DEFAULT_WALL_THICKNESS,
            mesh_resolution: defaults::DEFAULT_MESH_RESOLUTION,
            optimize_junctions: true,
            validate_quality: true,
            export_formats: vec![ExportFormat::VTK, ExportFormat::STL],
        }
    }
}

/// Main `Blue2Mesh` pipeline for complete workflow
pub struct Blue2MeshPipeline {
    config: Blue2MeshConfig,
}

impl Blue2MeshPipeline {
    /// Create new pipeline with configuration
    #[must_use] pub const fn new(config: Blue2MeshConfig) -> Self {
        Self { config }
    }

    /// Process scheme design to 3D mesh with full pipeline
    pub fn process_scheme_to_mesh<P1: AsRef<std::path::Path>, P2: AsRef<std::path::Path>>(
        &self,
        scheme_json_path: P1,
        output_directory: P2,
    ) -> crate::error::MeshResult<Vec<std::path::PathBuf>> {
        // Import scheme design
        let design = SchemeImporter::from_json_file(scheme_json_path)?;
        
        // Configure extrusion
        let extrusion_config = ExtrusionConfig::new()
            .with_height(self.config.channel_height)
            .with_wall_thickness(self.config.wall_thickness)
            .with_mesh_resolution(self.config.mesh_resolution);

        // Generate 3D mesh
        let mut mesh = ExtrusionEngine::new(extrusion_config)
            .extrude_design(&design)?;

        // Optimize junctions if enabled
        if self.config.optimize_junctions {
            mesh = junctions::JunctionOptimizer::new()
                .optimize_mesh(mesh)?;
        }

        // Validate quality if enabled
        if self.config.validate_quality {
            let quality_metrics = QualityController::new()
                .validate_mesh(&mesh)?;
            
            if quality_metrics.overall_score < 0.7 {
                log::warn!("Mesh quality score {:.2} is below recommended threshold", 
                          quality_metrics.overall_score);
            }
        }

        // Export in requested formats
        let mut output_files = Vec::new();
        for format in &self.config.export_formats {
            let filename = match format {
                ExportFormat::VTK => "device.vtk",
                ExportFormat::OpenFOAM => "polyMesh",
                ExportFormat::STL => "device.stl",
                ExportFormat::OBJ => "device.obj",
                _ => "device.mesh",
            };
            
            let output_path = output_directory.as_ref().join(filename);
            
            match format {
                ExportFormat::VTK => {
                    CfdExporter::new().export_vtk(&mesh, &output_path)?;
                }
                ExportFormat::STL => {
                    ManufacturingExporter::new().export_stl(&mesh, &output_path)?;
                }
                _ => {
                    log::warn!("Export format {format:?} not yet implemented");
                }
            }
            
            output_files.push(output_path);
        }

        Ok(output_files)
    }
}

/// Quick-start functions for common use cases
pub mod quickstart {
    use super::{Blue2MeshConfig, QualityLevel, ExportFormat, Blue2MeshPipeline};
    use crate::extrusion::ExtrusionConfig;

    /// Generate CFD-ready mesh from scheme JSON with default settings
    pub fn scheme_to_cfd_mesh<P1: AsRef<std::path::Path>, P2: AsRef<std::path::Path>>(
        scheme_json: P1,
        output_vtk: P2,
    ) -> crate::error::MeshResult<()> {
        let config = Blue2MeshConfig {
            quality_level: QualityLevel::HighQuality,
            export_formats: vec![ExportFormat::VTK],
            ..Default::default()
        };

        let pipeline = Blue2MeshPipeline::new(config);
        let output_dir = output_vtk.as_ref().parent()
            .ok_or_else(|| crate::error::MeshError::invalid_input("Invalid output path"))?;

        let _outputs = pipeline.process_scheme_to_mesh(scheme_json, output_dir)?;
        Ok(())
    }

    /// Generate manufacturing-ready STL from scheme JSON using pure CSG
    pub fn scheme_to_manufacturing<P1: AsRef<std::path::Path>, P2: AsRef<std::path::Path>>(
        scheme_json: P1,
        output_stl: P2,
    ) -> crate::error::MeshResult<()> {
        let extrusion_config = ExtrusionConfig::new()
            .with_csg_operations(true); // Force CSG operations
        scheme_to_manufacturing_with_config(scheme_json, output_stl, extrusion_config)
    }

    /// Generate manufacturing-ready STL from scheme JSON using pure CSG
    /// with explicit extrusion/CSG configuration.
    pub fn scheme_to_manufacturing_with_config<P1: AsRef<std::path::Path>, P2: AsRef<std::path::Path>>(
        scheme_json: P1,
        output_stl: P2,
        extrusion_config: ExtrusionConfig,
    ) -> crate::error::MeshResult<()> {
        // Import scheme design
        let design = crate::import::SchemeImporter::from_json_file(scheme_json)?;

        // Generate CSG mesh directly
        let mesh_builder = crate::extrusion::Mesh3DBuilder::new(&extrusion_config);
        let csg_mesh = mesh_builder.build_with_csg_pure(&design)?;

        // Export directly to STL using pure CSG
        crate::export::ManufacturingExporter::export_csg_stl(&csg_mesh, &output_stl)?;

        Ok(())
    }
}
