//! 3D mesh representation and operations
//!
//! This module provides the core 3D mesh data structure and operations
//! for millifluidic device meshes, with integration to csgrs and CFD
//! export capabilities.

use crate::error::MeshResult;
use crate::import::DesignData;
use nalgebra::Point3;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// 3D mesh representation for millifluidic devices
#[derive(Debug, Clone)]
pub struct Mesh3D {
    /// Mesh vertices
    pub vertices: Vec<Point3<f64>>,
    /// Triangular faces (indices into vertices)
    pub faces: Vec<[usize; 3]>,
    /// Channel region mapping (channel ID -> face indices)
    pub channel_regions: HashMap<usize, Vec<usize>>,
    /// Junction region mapping (junction ID -> face indices)
    pub junction_regions: HashMap<usize, Vec<usize>>,
    /// Mesh metadata
    pub metadata: MeshMetadata,
}

/// Metadata for 3D mesh
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct MeshMetadata {
    /// Mesh name
    pub name: String,
    /// Generation timestamp
    pub created_at: String,
    /// Mesh quality score (0-1)
    pub quality_score: f64,
    /// Total volume (m³)
    pub volume: f64,
    /// Total surface area (m²)
    pub surface_area: f64,
    /// Mesh resolution (m)
    pub resolution: f64,
    /// Whether mesh is manifold
    pub is_manifold: bool,
    /// Whether mesh is CFD-ready
    pub cfd_ready: bool,
    /// Generation parameters
    pub generation_params: HashMap<String, String>,
}

/// Configuration for mesh generation
#[derive(Debug, Clone)]
pub struct MeshConfig {
    /// Target mesh resolution (m)
    pub resolution: f64,
    /// Quality level
    pub quality_level: crate::QualityLevel,
    /// Mesh strategy
    pub strategy: crate::MeshStrategy,
    /// Whether to optimize for CFD
    pub cfd_optimized: bool,
    /// Whether to generate boundary layers
    pub boundary_layers: bool,
    /// Number of boundary layer elements
    pub boundary_layer_count: usize,
    /// Boundary layer growth ratio
    pub boundary_layer_ratio: f64,
}

/// Main mesh generator
pub struct MeshGenerator {
    config: MeshConfig,
}

impl MeshGenerator {
    /// Create new mesh generator
    pub fn new() -> Self {
        Self {
            config: MeshConfig::default(),
        }
    }

    /// Set mesh configuration
    pub fn with_config(mut self, config: MeshConfig) -> Self {
        self.config = config;
        self
    }

    /// Set mesh resolution
    pub fn with_resolution(mut self, resolution: f64) -> Self {
        self.config.resolution = resolution;
        self
    }

    /// Set quality level
    pub fn with_quality_level(mut self, level: crate::QualityLevel) -> Self {
        self.config.quality_level = level;
        self
    }

    /// Generate mesh from scheme JSON file
    pub fn generate_from_scheme<P: AsRef<std::path::Path>>(&self, json_path: P) -> MeshResult<Mesh3D> {
        let design = crate::import::SchemeImporter::from_json_file(json_path)?;
        self.generate_from_design(&design)
    }

    /// Generate mesh from design data
    pub fn generate_from_design(&self, design: &DesignData) -> MeshResult<Mesh3D> {
        // Configure extrusion based on mesh config
        let extrusion_config = crate::extrusion::ExtrusionConfig::new()
            .with_mesh_resolution(self.config.resolution)
            .with_strategy(self.map_strategy_to_extrusion());

        // Generate base mesh through extrusion
        let extrusion_engine = crate::extrusion::ExtrusionEngine::new(extrusion_config);
        let mut mesh = extrusion_engine.extrude_design(design)?;

        // Apply mesh strategy-specific optimizations
        match self.config.strategy {
            crate::MeshStrategy::Structured => {
                self.apply_structured_optimization(&mut mesh)?;
            }
            crate::MeshStrategy::Unstructured => {
                self.apply_unstructured_optimization(&mut mesh)?;
            }
            crate::MeshStrategy::Hybrid => {
                self.apply_hybrid_optimization(&mut mesh)?;
            }
            crate::MeshStrategy::Adaptive => {
                self.apply_adaptive_refinement(&mut mesh)?;
            }
        }

        // Generate boundary layers if requested
        if self.config.boundary_layers {
            self.generate_boundary_layers(&mut mesh)?;
        }

        // Update metadata
        mesh.metadata.resolution = self.config.resolution;
        mesh.metadata.cfd_ready = self.config.cfd_optimized;
        mesh.metadata.created_at = chrono::Utc::now().to_rfc3339();

        Ok(mesh)
    }

    /// Map mesh strategy to extrusion strategy
    fn map_strategy_to_extrusion(&self) -> crate::extrusion::ExtrusionStrategy {
        match self.config.strategy {
            crate::MeshStrategy::Structured => crate::extrusion::ExtrusionStrategy::Linear,
            crate::MeshStrategy::Unstructured => crate::extrusion::ExtrusionStrategy::Swept,
            crate::MeshStrategy::Hybrid => crate::extrusion::ExtrusionStrategy::Lofted,
            crate::MeshStrategy::Adaptive => crate::extrusion::ExtrusionStrategy::Adaptive,
        }
    }

    /// Apply structured mesh optimization
    fn apply_structured_optimization(&self, _mesh: &mut Mesh3D) -> MeshResult<()> {
        log::info!("Applying structured mesh optimization");
        // Placeholder for structured mesh optimization
        Ok(())
    }

    /// Apply unstructured mesh optimization
    fn apply_unstructured_optimization(&self, _mesh: &mut Mesh3D) -> MeshResult<()> {
        log::info!("Applying unstructured mesh optimization");
        // Placeholder for unstructured mesh optimization
        Ok(())
    }

    /// Apply hybrid mesh optimization
    fn apply_hybrid_optimization(&self, _mesh: &mut Mesh3D) -> MeshResult<()> {
        log::info!("Applying hybrid mesh optimization");
        // Placeholder for hybrid mesh optimization
        Ok(())
    }

    /// Apply adaptive mesh refinement
    fn apply_adaptive_refinement(&self, _mesh: &mut Mesh3D) -> MeshResult<()> {
        log::info!("Applying adaptive mesh refinement");
        // Placeholder for adaptive refinement
        Ok(())
    }

    /// Generate boundary layers for CFD
    fn generate_boundary_layers(&self, _mesh: &mut Mesh3D) -> MeshResult<()> {
        log::info!("Generating {} boundary layers with ratio {:.2}", 
                  self.config.boundary_layer_count, self.config.boundary_layer_ratio);
        // Placeholder for boundary layer generation
        Ok(())
    }
}

impl Mesh3D {
    /// Create new empty mesh
    pub fn new() -> Self {
        Self {
            vertices: Vec::new(),
            faces: Vec::new(),
            channel_regions: HashMap::new(),
            junction_regions: HashMap::new(),
            metadata: MeshMetadata::default(),
        }
    }

    /// Calculate mesh volume
    pub fn calculate_volume(&self) -> f64 {
        let mut total_volume = 0.0;

        for face in &self.faces {
            let v1 = self.vertices[face[0]];
            let v2 = self.vertices[face[1]];
            let v3 = self.vertices[face[2]];

            // Calculate tetrahedron volume with origin
            let volume = (v1.coords.dot(&v2.coords.cross(&v3.coords))) / 6.0;
            total_volume += volume;
        }

        total_volume.abs()
    }

    /// Calculate mesh surface area
    pub fn calculate_surface_area(&self) -> f64 {
        let mut total_area = 0.0;

        for face in &self.faces {
            let v1 = self.vertices[face[0]];
            let v2 = self.vertices[face[1]];
            let v3 = self.vertices[face[2]];

            // Calculate triangle area
            let edge1 = v2 - v1;
            let edge2 = v3 - v1;
            let area = edge1.cross(&edge2).norm() / 2.0;
            total_area += area;
        }

        total_area
    }

    /// Get mesh bounds
    pub fn bounds(&self) -> (Point3<f64>, Point3<f64>) {
        if self.vertices.is_empty() {
            return (Point3::origin(), Point3::origin());
        }

        let mut min_point = self.vertices[0];
        let mut max_point = self.vertices[0];

        for vertex in &self.vertices {
            min_point.x = min_point.x.min(vertex.x);
            min_point.y = min_point.y.min(vertex.y);
            min_point.z = min_point.z.min(vertex.z);
            
            max_point.x = max_point.x.max(vertex.x);
            max_point.y = max_point.y.max(vertex.y);
            max_point.z = max_point.z.max(vertex.z);
        }

        (min_point, max_point)
    }

    /// Convert to csgrs mesh for boolean operations
    pub fn to_csgrs_mesh(&self) -> MeshResult<csgrs::mesh::Mesh<String>> {
        use csgrs::mesh::polygon::Polygon;
        use csgrs::mesh::vertex::Vertex;
        use csgrs::float_types::Real;
        use csgrs::float_types::parry3d::na::{Point3 as CsgrsPoint3, Vector3 as CsgrsVector3};

        let mut polygons = Vec::new();

        // Convert faces to csgrs polygons
        for face in &self.faces {
            let vertices: Vec<Vertex> = face.iter()
                .map(|&idx| {
                    let pos = self.vertices[idx];
                    // Calculate face normal from vertices
                    let v0 = self.vertices[face[0]];
                    let v1 = self.vertices[face[1]];
                    let v2 = self.vertices[face[2]];
                    let edge1 = v1 - v0;
                    let edge2 = v2 - v0;
                    let normal = edge1.cross(&edge2).normalize();

                    Vertex::new(
                        CsgrsPoint3::new(pos.x as Real, pos.y as Real, pos.z as Real),
                        CsgrsVector3::new(normal.x as Real, normal.y as Real, normal.z as Real),
                    )
                })
                .collect();

            if vertices.len() >= 3 {
                polygons.push(Polygon::new(vertices, None));
            }
        }

        Ok(csgrs::mesh::Mesh::from_polygons(&polygons, None))
    }

    /// Create from csgrs mesh
    pub fn from_csgrs_mesh(csgrs_mesh: &csgrs::mesh::Mesh<String>) -> MeshResult<Self> {
        let mut vertices = Vec::new();
        let mut faces = Vec::new();

        // Extract vertices and faces from csgrs polygons
        // For simplicity, we'll not deduplicate vertices for now
        for polygon in &csgrs_mesh.polygons {
            let mut face_vertices = Vec::new();

            for vertex in &polygon.vertices {
                let pos = Point3::new(
                    vertex.pos.x as f64,
                    vertex.pos.y as f64,
                    vertex.pos.z as f64,
                );

                let vertex_idx = vertices.len();
                vertices.push(pos);
                face_vertices.push(vertex_idx);
            }

            // Convert polygon to triangular faces if needed
            if face_vertices.len() >= 3 {
                // For triangles, add directly
                if face_vertices.len() == 3 {
                    faces.push([face_vertices[0], face_vertices[1], face_vertices[2]]);
                } else {
                    // For polygons with more than 3 vertices, triangulate
                    for i in 1..face_vertices.len() - 1 {
                        faces.push([face_vertices[0], face_vertices[i], face_vertices[i + 1]]);
                    }
                }
            }
        }

        Ok(Self {
            vertices,
            faces,
            channel_regions: HashMap::new(),
            junction_regions: HashMap::new(),
            metadata: MeshMetadata::default(),
        })
    }
}

impl Default for MeshConfig {
    fn default() -> Self {
        Self {
            resolution: crate::defaults::DEFAULT_MESH_RESOLUTION,
            quality_level: crate::QualityLevel::Balanced,
            strategy: crate::MeshStrategy::Hybrid,
            cfd_optimized: true,
            boundary_layers: false,
            boundary_layer_count: 3,
            boundary_layer_ratio: 1.2,
        }
    }
}

impl Default for MeshGenerator {
    fn default() -> Self {
        Self::new()
    }
}

impl Default for Mesh3D {
    fn default() -> Self {
        Self::new()
    }
}
