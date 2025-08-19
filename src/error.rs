//! Error handling for the Blue2Mesh framework
//!
//! This module provides comprehensive error types for all mesh generation
//! operations, following Rust best practices for error handling and providing
//! detailed context for debugging and user feedback.

use thiserror::Error;
use serde_json;

/// Result type alias for Blue2Mesh operations
pub type MeshResult<T> = Result<T, MeshError>;

/// Comprehensive error types for Blue2Mesh operations
#[derive(Error, Debug)]
pub enum MeshError {
    /// Invalid input data or parameters
    #[error("Invalid input: {message}")]
    InvalidInput { message: String },

    /// Mesh generation errors
    #[error("Mesh generation error: {message}")]
    MeshGeneration { message: String },

    /// Extrusion algorithm errors
    #[error("Extrusion error: {message}")]
    ExtrusionError { message: String },

    /// Mesh quality validation errors
    #[error("Mesh quality error: {message}")]
    QualityError { message: String },

    /// Junction optimization errors
    #[error("Junction optimization error: {message}")]
    JunctionError { message: String },

    /// Scheme integration errors
    #[error("Scheme integration error: {message}")]
    SchemeIntegrationError { message: String },

    /// Export format errors
    #[error("Export error: {message}")]
    ExportError { message: String },

    /// Geometric computation errors
    #[error("Geometry error: {message}")]
    GeometryError { message: String },

    /// File I/O errors
    #[error("I/O error: {source}")]
    IoError {
        #[from]
        source: std::io::Error,
    },

    /// JSON parsing errors
    #[error("JSON error: {source}")]
    JsonError {
        #[from]
        source: serde_json::Error,
    },

    /// CSG operation errors from csgrs
    #[error("CSG error: {message}")]
    CsgError { message: String },

    /// Memory allocation errors for large meshes
    #[error("Memory error: {message}")]
    MemoryError { message: String },

    /// Numerical computation errors
    #[error("Numerical error: {message}")]
    NumericalError { message: String },
}

impl MeshError {
    /// Create an invalid input error
    pub fn invalid_input(message: impl Into<String>) -> Self {
        Self::InvalidInput {
            message: message.into(),
        }
    }

    /// Create a mesh generation error
    pub fn mesh_generation(message: impl Into<String>) -> Self {
        Self::MeshGeneration {
            message: message.into(),
        }
    }

    /// Create an extrusion error
    pub fn extrusion_error(message: impl Into<String>) -> Self {
        Self::ExtrusionError {
            message: message.into(),
        }
    }

    /// Create a quality error
    pub fn quality_error(message: impl Into<String>) -> Self {
        Self::QualityError {
            message: message.into(),
        }
    }

    /// Create a junction error
    pub fn junction_error(message: impl Into<String>) -> Self {
        Self::JunctionError {
            message: message.into(),
        }
    }

    /// Create a scheme integration error
    pub fn scheme_integration_error(message: impl Into<String>) -> Self {
        Self::SchemeIntegrationError {
            message: message.into(),
        }
    }

    /// Create an export error
    pub fn export_error(message: impl Into<String>) -> Self {
        Self::ExportError {
            message: message.into(),
        }
    }

    /// Create a geometry error
    pub fn geometry_error(message: impl Into<String>) -> Self {
        Self::GeometryError {
            message: message.into(),
        }
    }

    /// Create a CSG error
    pub fn csg_error(message: impl Into<String>) -> Self {
        Self::CsgError {
            message: message.into(),
        }
    }

    /// Create a memory error
    pub fn memory_error(message: impl Into<String>) -> Self {
        Self::MemoryError {
            message: message.into(),
        }
    }

    /// Create a numerical error
    pub fn numerical_error(message: impl Into<String>) -> Self {
        Self::NumericalError {
            message: message.into(),
        }
    }
}

/// Specialized error types for different mesh operations
pub mod specialized {
    use super::*;

    /// Errors specific to mesh quality validation
    #[derive(Error, Debug)]
    pub enum QualityValidationError {
        #[error("Aspect ratio {ratio:.2} exceeds limit {limit:.2}")]
        AspectRatioExceeded { ratio: f64, limit: f64 },
        
        #[error("Minimum angle {angle:.1}° below threshold {threshold:.1}°")]
        MinAngleBelowThreshold { angle: f64, threshold: f64 },
        
        #[error("Maximum angle {angle:.1}° exceeds threshold {threshold:.1}°")]
        MaxAngleExceedsThreshold { angle: f64, threshold: f64 },
        
        #[error("Mesh contains {count} degenerate elements")]
        DegenerateElements { count: usize },
        
        #[error("Mesh is not manifold: {issue}")]
        NonManifold { issue: String },
        
        #[error("Mesh contains {count} self-intersections")]
        SelfIntersections { count: usize },
    }

    /// Errors specific to extrusion operations
    #[derive(Error, Debug)]
    pub enum ExtrusionError {
        #[error("Invalid extrusion height: {height} (must be positive)")]
        InvalidHeight { height: f64 },
        
        #[error("Wall thickness {thickness} exceeds channel width {width}")]
        WallThicknessTooLarge { thickness: f64, width: f64 },
        
        #[error("Mesh resolution {resolution} is too coarse for geometry")]
        ResolutionTooCoarse { resolution: f64 },
        
        #[error("Channel path contains {count} invalid segments")]
        InvalidChannelPath { count: usize },
        
        #[error("Extrusion failed at junction {junction_id}: {reason}")]
        JunctionExtrusionFailed { junction_id: usize, reason: String },
    }

    /// Errors specific to export operations
    #[derive(Error, Debug)]
    pub enum ExportError {
        #[error("Unsupported export format: {format}")]
        UnsupportedFormat { format: String },
        
        #[error("Mesh validation failed before export: {reason}")]
        PreExportValidationFailed { reason: String },
        
        #[error("File write failed: {path}")]
        FileWriteFailed { path: String },
        
        #[error("Format conversion failed: {details}")]
        FormatConversionFailed { details: String },
    }
}

/// Error context for better debugging
#[derive(Debug, Clone)]
pub struct ErrorContext {
    pub operation: String,
    pub parameters: std::collections::HashMap<String, String>,
    pub timestamp: std::time::SystemTime,
    pub mesh_statistics: Option<MeshStatistics>,
}

/// Basic mesh statistics for error context
#[derive(Debug, Clone)]
pub struct MeshStatistics {
    pub vertex_count: usize,
    pub face_count: usize,
    pub volume: f64,
    pub surface_area: f64,
}

impl ErrorContext {
    pub fn new(operation: impl Into<String>) -> Self {
        Self {
            operation: operation.into(),
            parameters: std::collections::HashMap::new(),
            timestamp: std::time::SystemTime::now(),
            mesh_statistics: None,
        }
    }

    pub fn with_parameter(mut self, key: impl Into<String>, value: impl Into<String>) -> Self {
        self.parameters.insert(key.into(), value.into());
        self
    }

    pub fn with_mesh_stats(mut self, stats: MeshStatistics) -> Self {
        self.mesh_statistics = Some(stats);
        self
    }
}

/// Extension trait for adding context to errors
pub trait ErrorContextExt<T> {
    fn with_context(self, context: ErrorContext) -> MeshResult<T>;
}

impl<T> ErrorContextExt<T> for MeshResult<T> {
    fn with_context(self, context: ErrorContext) -> MeshResult<T> {
        self.map_err(|e| {
            eprintln!("Error context: {:?}", context);
            e
        })
    }
}

/// Convert csgrs errors to mesh errors
impl From<csgrs::io::IoError> for MeshError {
    fn from(err: csgrs::io::IoError) -> Self {
        Self::CsgError {
            message: format!("CSG operation failed: {:?}", err),
        }
    }
}

/// Convert scheme errors to mesh errors
impl From<scheme::error::SchemeError> for MeshError {
    fn from(err: scheme::error::SchemeError) -> Self {
        Self::SchemeIntegrationError {
            message: format!("Scheme integration failed: {:?}", err),
        }
    }
}
