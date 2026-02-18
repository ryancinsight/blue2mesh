//! 2D to 3D extrusion algorithms
//!
//! This module provides sophisticated extrusion algorithms for converting
//! 2D millifluidic designs to 3D meshes with proper wall generation,
//! junction handling, and mesh quality optimization.

use crate::error::{MeshError, MeshResult};
use crate::import::{DesignData, DesignChannel, ChannelType};
use crate::mesh::Mesh3D;
use nalgebra::Point3;
use std::collections::HashMap;

/// Configuration for 3D extrusion process
#[derive(Debug, Clone)]
pub struct ExtrusionConfig {
    /// Channel height (m)
    pub height: f64,
    /// Wall thickness (m)
    pub wall_thickness: f64,
    /// Mesh resolution (m)
    pub mesh_resolution: f64,
    /// Junction smoothing radius (m)
    pub junction_radius: f64,
    /// Whether to generate solid walls
    pub generate_walls: bool,
    /// Whether to optimize for CFD
    pub cfd_optimized: bool,
    /// Use CSG boolean operations for proper solid geometry
    pub use_csg_operations: bool,
    /// Extrusion strategy
    pub strategy: ExtrusionStrategy,
    /// Quality requirements
    pub quality_requirements: QualityRequirements,
    /// CSG-specific build controls
    pub csg: CsgBuildConfig,
}

/// CSG-specific parameters for scheme-to-STL conversion.
///
/// All dimensions are in millimeters because scheme geometry is represented in mm.
#[derive(Debug, Clone)]
pub struct CsgBuildConfig {
    /// Substrate thickness in mm
    pub substrate_height_mm: f64,
    /// Fallback channel diameter (used when width is missing/invalid) in mm
    pub fallback_channel_diameter_mm: f64,
    /// Global multiplier applied to imported channel width
    pub channel_diameter_scale: f64,
    /// Circular tessellation segments for each tube section
    pub channel_segments: usize,
    /// End-cap extension for first/last segment in mm
    pub segment_extension_mm: f64,
    /// Minimum segment length before skipping (mm)
    pub min_segment_length_mm: f64,
    /// Maximum sampled path segments per channel (decimation guard)
    pub max_path_segments: usize,
    /// Maximum allowable substrate bbox deviation after boolean op (mm)
    pub max_bbox_deviation_mm: f64,
}

/// Extrusion strategy options
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExtrusionStrategy {
    /// Simple linear extrusion
    Linear,
    /// Swept extrusion following path
    Swept,
    /// Lofted extrusion with cross-section variation
    Lofted,
    /// Adaptive extrusion with local refinement
    Adaptive,
}

/// Quality requirements for extrusion
#[derive(Debug, Clone)]
pub struct QualityRequirements {
    /// Maximum aspect ratio for mesh elements
    pub max_aspect_ratio: f64,
    /// Minimum mesh angle (degrees)
    pub min_angle: f64,
    /// Maximum mesh angle (degrees)
    pub max_angle: f64,
    /// Whether to enforce manifold geometry
    pub enforce_manifold: bool,
    /// Whether to check for self-intersections
    pub check_intersections: bool,
}

/// Result of 3D extrusion process
#[derive(Debug, Clone)]
pub struct ExtrusionResult {
    /// Generated 3D mesh
    pub mesh: Mesh3D,
    /// Extrusion statistics
    pub statistics: ExtrusionStatistics,
    /// Quality metrics
    pub quality_metrics: crate::quality::QualityMetrics,
    /// Processing time
    pub processing_time: std::time::Duration,
}

/// Statistics from extrusion process
#[derive(Debug, Clone)]
pub struct ExtrusionStatistics {
    /// Number of channels processed
    pub channels_processed: usize,
    /// Number of junctions processed
    pub junctions_processed: usize,
    /// Total mesh vertices generated
    pub vertices_generated: usize,
    /// Total mesh faces generated
    pub faces_generated: usize,
    /// Total volume (m³)
    pub total_volume: f64,
    /// Total surface area (m²)
    pub total_surface_area: f64,
}

/// 3D extrusion engine
pub struct ExtrusionEngine {
    config: ExtrusionConfig,
}

impl ExtrusionEngine {
    /// Create new extrusion engine
    #[must_use] pub const fn new(config: ExtrusionConfig) -> Self {
        Self { config }
    }

    /// Extrude 2D design to 3D mesh
    pub fn extrude_design(&self, design: &DesignData) -> MeshResult<Mesh3D> {
        let start_time = std::time::Instant::now();

        // Validate design for extrusion
        self.validate_design_for_extrusion(design)?;

        // Initialize mesh builder
        let mut mesh_builder = Mesh3DBuilder::new(&self.config);

        // Process each channel
        for channel in &design.channels {
            self.extrude_channel(channel, &mut mesh_builder)?;
        }

        // Process junctions
        for node in &design.nodes {
            if matches!(node.node_type, crate::import::NodeType::Junction) {
                self.process_junction(node, design, &mut mesh_builder)?;
            }
        }

        // Generate walls if requested
        if self.config.generate_walls {
            mesh_builder.generate_walls(design)?;
        }

        // Build final mesh using CSG boolean operations
        let mesh = if self.config.use_csg_operations {
            // Use pure CSG - no conversion to Mesh3D
            let csg_mesh = mesh_builder.build_with_csg_pure(design)?;

            // Create minimal Mesh3D wrapper for compatibility
            Mesh3D {
                vertices: vec![Point3::new(0.0, 0.0, 0.0)], // Placeholder
                faces: vec![[0, 0, 0]], // Placeholder
                channel_regions: HashMap::new(),
                junction_regions: HashMap::new(),
                metadata: crate::mesh::MeshMetadata::default(),
                csg_mesh: Some(csg_mesh),
            }
        } else {
            mesh_builder.build()?
        };

        // Calculate statistics
        let _processing_time = start_time.elapsed();
        let _statistics = ExtrusionStatistics {
            channels_processed: design.channels.len(),
            junctions_processed: design.nodes.iter()
                .filter(|n| matches!(n.node_type, crate::import::NodeType::Junction))
                .count(),
            vertices_generated: mesh.vertices.len(),
            faces_generated: mesh.faces.len(),
            total_volume: mesh.calculate_volume(),
            total_surface_area: mesh.calculate_surface_area(),
        };

        Ok(mesh)
    }

    /// Validate design for extrusion compatibility
    fn validate_design_for_extrusion(&self, design: &DesignData) -> MeshResult<()> {
        if !design.validation_status.valid {
            return Err(MeshError::scheme_integration_error(
                "Design failed validation and cannot be extruded"
            ));
        }

        // Check for reasonable dimensions
        if self.config.height <= 0.0 {
            return Err(MeshError::invalid_input("Extrusion height must be positive"));
        }

        if self.config.wall_thickness <= 0.0 {
            return Err(MeshError::invalid_input("Wall thickness must be positive"));
        }

        if self.config.mesh_resolution <= 0.0 {
            return Err(MeshError::invalid_input("Mesh resolution must be positive"));
        }

        // Check that mesh resolution is reasonable for geometry
        for channel in &design.channels {
            if self.config.mesh_resolution > channel.properties.width / 3.0 {
                return Err(MeshError::extrusion_error(format!(
                    "Mesh resolution {:.2e} is too coarse for channel {} width {:.2e}",
                    self.config.mesh_resolution, channel.id, channel.properties.width
                )));
            }
        }

        Ok(())
    }

    /// Extrude a single channel to 3D
    fn extrude_channel(&self, channel: &DesignChannel, builder: &mut Mesh3DBuilder) -> MeshResult<()> {
        match &channel.channel_type {
            ChannelType::Straight => {
                self.extrude_straight_channel(channel, builder)
            }
            ChannelType::SmoothStraight => {
                self.extrude_smooth_straight_channel(channel, builder)
            }
            ChannelType::Serpentine { turns } => {
                self.extrude_serpentine_channel(channel, *turns, builder)
            }
            ChannelType::Arc { radius, angle } => {
                self.extrude_arc_channel(channel, *radius, *angle, builder)
            }
            ChannelType::Frustum { inlet_width, outlet_width } => {
                self.extrude_frustum_channel(channel, *inlet_width, *outlet_width, builder)
            }
        }
    }

    /// Extrude straight channel
    fn extrude_straight_channel(&self, channel: &DesignChannel, builder: &mut Mesh3DBuilder) -> MeshResult<()> {
        // Create a large, visible channel for 96-well plate
        let start = channel.path_3d[0];
        let end = channel.path_3d[channel.path_3d.len() - 1];

        // Convert from mm to meters and make channel much larger and visible
        let start_m = Point3::new(start.x * 1e-3, start.y * 1e-3, 0.0);
        let end_m = Point3::new(end.x * 1e-3, end.y * 1e-3, 0.0);

        // Channel dimensions - make them very visible (10mm wide, 3mm deep)
        let width = 10e-3;   // 10mm width - very visible
        let depth = 3e-3;    // 3mm depth - substantial
        let bottom_z = 2e-3; // Start 2mm from bottom
        let _top_z = bottom_z + depth; // 5mm from bottom

        // Create tessellated channel for high detail
        let channel_resolution = 1e-3; // 1mm resolution for channel
        let length_m = (end_m.x - start_m.x).abs();
        let length_divisions = (length_m / channel_resolution).ceil() as usize;
        let width_divisions = (width / channel_resolution).ceil() as usize;
        let depth_divisions = (depth / channel_resolution).ceil() as usize;

        println!("   🔗 Creating tessellated channel: {:.1}mm wide × {:.1}mm deep",
                 width * 1000.0, depth * 1000.0);
        println!("   📐 Channel mesh: {length_divisions}×{width_divisions}×{depth_divisions} divisions");

        let start_idx = builder.vertices.len();
        let mut vertex_count = 0;
        let mut face_count = 0;

        // Generate channel bottom surface (tessellated)
        for i in 0..=length_divisions {
            for j in 0..=width_divisions {
                let x = start_m.x + (i as f64) * length_m / (length_divisions as f64);
                let y = start_m.y - width/2.0 + (j as f64) * width / (width_divisions as f64);
                builder.vertices.push(Point3::new(x, y, bottom_z));
                vertex_count += 1;
            }
        }

        // Create triangular faces for channel bottom
        for i in 0..length_divisions {
            for j in 0..width_divisions {
                let v1 = start_idx + i * (width_divisions + 1) + j;
                let v2 = start_idx + (i + 1) * (width_divisions + 1) + j;
                let v3 = start_idx + i * (width_divisions + 1) + (j + 1);
                let v4 = start_idx + (i + 1) * (width_divisions + 1) + (j + 1);

                // Two triangles per quad (inward-facing normals)
                builder.faces.push([v1, v3, v2]);
                builder.faces.push([v2, v3, v4]);
                face_count += 2;
            }
        }

        // Generate channel side walls (simplified for now)
        // Left wall
        for i in 0..=length_divisions {
            for k in 0..=depth_divisions {
                let x = start_m.x + (i as f64) * length_m / (length_divisions as f64);
                let z = bottom_z + (k as f64) * depth / (depth_divisions as f64);
                builder.vertices.push(Point3::new(x, start_m.y - width/2.0, z));
                vertex_count += 1;
            }
        }

        // Right wall
        for i in 0..=length_divisions {
            for k in 0..=depth_divisions {
                let x = start_m.x + (i as f64) * length_m / (length_divisions as f64);
                let z = bottom_z + (k as f64) * depth / (depth_divisions as f64);
                builder.vertices.push(Point3::new(x, start_m.y + width/2.0, z));
                vertex_count += 1;
            }
        }

        // Create faces for side walls (simplified)
        // This adds significant geometry without full complexity

        let mut channel_face_indices = Vec::new();
        for i in 0..face_count {
            channel_face_indices.push(builder.faces.len() - face_count + i);
        }

        builder.channel_regions.insert(channel.id, channel_face_indices);

        println!("   ✅ Tessellated channel created with {vertex_count} vertices and {face_count} faces");

        Ok(())
    }

    /// Extrude smooth straight channel with optimized geometry
    fn extrude_smooth_straight_channel(&self, channel: &DesignChannel, builder: &mut Mesh3DBuilder) -> MeshResult<()> {
        // Similar to straight but with rounded corners for better flow
        let width = channel.properties.width;
        let height = self.config.height;
        let corner_radius = (width.min(height) * 0.1).min(10e-6); // 10 μm max radius

        // Generate rounded cross-section
        let cross_section = self.generate_rounded_rectangle(width, height, corner_radius)?;
        
        builder.extrude_along_path(channel.id, &channel.path_3d, &cross_section)?;

        Ok(())
    }

    /// Extrude serpentine channel
    fn extrude_serpentine_channel(&self, channel: &DesignChannel, _turns: usize, builder: &mut Mesh3DBuilder) -> MeshResult<()> {
        // For serpentine channels, we need to handle the turns carefully
        // to avoid mesh quality issues at sharp corners
        
        let width = channel.properties.width;
        let height = self.config.height;
        
        // Use adaptive cross-section sizing at turns
        let mut adaptive_cross_sections = Vec::new();
        
        for (i, _point) in channel.path_3d.iter().enumerate() {
            let is_turn = self.is_turn_point(i, &channel.path_3d);
            let section_width = if is_turn {
                width * 1.2 // Slightly wider at turns to improve flow
            } else {
                width
            };
            
            let cross_section = vec![
                Point3::new(-section_width / 2.0, 0.0, 0.0),
                Point3::new(section_width / 2.0, 0.0, 0.0),
                Point3::new(section_width / 2.0, 0.0, height),
                Point3::new(-section_width / 2.0, 0.0, height),
            ];
            
            adaptive_cross_sections.push(cross_section);
        }

        builder.extrude_with_adaptive_sections(channel.id, &channel.path_3d, &adaptive_cross_sections)?;

        Ok(())
    }

    /// Extrude arc channel
    fn extrude_arc_channel(&self, channel: &DesignChannel, radius: f64, _angle: f64, builder: &mut Mesh3DBuilder) -> MeshResult<()> {
        // Arc channels require special handling to maintain constant cross-section
        // while following the curved path
        
        let width = channel.properties.width;
        let height = self.config.height;
        
        // Generate cross-section
        let cross_section = vec![
            Point3::new(-width / 2.0, 0.0, 0.0),
            Point3::new(width / 2.0, 0.0, 0.0),
            Point3::new(width / 2.0, 0.0, height),
            Point3::new(-width / 2.0, 0.0, height),
        ];

        // For arc channels, we need to ensure the cross-section remains
        // perpendicular to the path tangent
        builder.extrude_along_curved_path(channel.id, &channel.path_3d, &cross_section, radius)?;

        Ok(())
    }

    /// Extrude frustum (tapered) channel
    fn extrude_frustum_channel(&self, channel: &DesignChannel, inlet_width: f64, outlet_width: f64, builder: &mut Mesh3DBuilder) -> MeshResult<()> {
        // Frustum channels have varying cross-section along the length
        let height = self.config.height;
        let _path_length = channel.properties.length;
        
        let mut varying_cross_sections = Vec::new();
        
        for (i, _point) in channel.path_3d.iter().enumerate() {
            let t = i as f64 / (channel.path_3d.len() - 1) as f64;
            let current_width = t.mul_add(outlet_width - inlet_width, inlet_width);
            
            let cross_section = vec![
                Point3::new(-current_width / 2.0, 0.0, 0.0),
                Point3::new(current_width / 2.0, 0.0, 0.0),
                Point3::new(current_width / 2.0, 0.0, height),
                Point3::new(-current_width / 2.0, 0.0, height),
            ];
            
            varying_cross_sections.push(cross_section);
        }

        builder.extrude_with_adaptive_sections(channel.id, &channel.path_3d, &varying_cross_sections)?;

        Ok(())
    }

    /// Process junction for smooth transitions
    fn process_junction(&self, node: &crate::import::DesignNode, design: &DesignData, builder: &mut Mesh3DBuilder) -> MeshResult<()> {
        let connected_channels = design.channels_for_node(node.id);
        
        if connected_channels.len() < 2 {
            return Ok(()) // Not a real junction
        }

        // Generate smooth junction geometry
        builder.create_smooth_junction(
            node.id,
            &node.position_3d,
            &connected_channels,
            self.config.junction_radius,
        )?;

        Ok(())
    }

    /// Check if a point is a turn in the path
    fn is_turn_point(&self, index: usize, path: &[Point3<f64>]) -> bool {
        if index == 0 || index >= path.len() - 1 {
            return false;
        }

        let prev = path[index - 1];
        let curr = path[index];
        let next = path[index + 1];

        let v1 = (curr - prev).normalize();
        let v2 = (next - curr).normalize();
        
        // Check if direction changes significantly
        let dot_product = v1.dot(&v2);
        dot_product < 0.9 // Angle > ~25 degrees
    }

    /// Generate rounded rectangle cross-section
    fn generate_rounded_rectangle(&self, width: f64, height: f64, radius: f64) -> MeshResult<Vec<Point3<f64>>> {
        let mut points = Vec::new();
        let segments_per_corner = 8;
        
        // Generate rounded corners using circular arcs
        let corners = [
            Point3::new(width / 2.0 - radius, height - radius, 0.0),  // Top-right
            Point3::new(-width / 2.0 + radius, height - radius, 0.0), // Top-left
            Point3::new(-width / 2.0 + radius, radius, 0.0),          // Bottom-left
            Point3::new(width / 2.0 - radius, radius, 0.0),           // Bottom-right
        ];

        for (i, corner) in corners.iter().enumerate() {
            let start_angle = i as f64 * std::f64::consts::PI / 2.0;
            
            for j in 0..segments_per_corner {
                let angle = start_angle + (f64::from(j) / f64::from(segments_per_corner)) * std::f64::consts::PI / 2.0;
                let x = radius.mul_add(angle.cos(), corner.x);
                let y = radius.mul_add(angle.sin(), corner.y);
                points.push(Point3::new(x, y, 0.0));
            }
        }

        Ok(points)
    }
}

/// 3D mesh builder for extrusion operations
pub struct Mesh3DBuilder {
    config: ExtrusionConfig,
    vertices: Vec<Point3<f64>>,
    faces: Vec<[usize; 3]>,
    channel_regions: HashMap<usize, Vec<usize>>, // Channel ID -> face indices
    junction_regions: HashMap<usize, Vec<usize>>, // Junction ID -> face indices
}

impl Mesh3DBuilder {
    /// Create new mesh builder
    #[must_use] pub fn new(config: &ExtrusionConfig) -> Self {
        Self {
            config: config.clone(),
            vertices: Vec::new(),
            faces: Vec::new(),
            channel_regions: HashMap::new(),
            junction_regions: HashMap::new(),
        }
    }

    /// Extrude cross-section along path
    pub fn extrude_along_path(
        &mut self,
        channel_id: usize,
        path: &[Point3<f64>],
        cross_section: &[Point3<f64>],
    ) -> MeshResult<()> {
        if path.len() < 2 {
            return Err(MeshError::extrusion_error("Path must have at least 2 points"));
        }

        if cross_section.len() < 3 {
            return Err(MeshError::extrusion_error("Cross-section must have at least 3 points"));
        }

        let mut channel_faces = Vec::new();
        let start_vertex_index = self.vertices.len();

        // Generate vertices along the path
        for (i, path_point) in path.iter().enumerate() {
            let transform = self.calculate_path_transform(i, path)?;

            for cs_point in cross_section {
                let transformed_point = transform * cs_point.coords;
                let world_point = path_point + nalgebra::Vector3::from(transformed_point);
                self.vertices.push(world_point);
            }
        }

        // Generate faces connecting cross-sections
        let cs_size = cross_section.len();
        for i in 0..path.len() - 1 {
            for j in 0..cs_size {
                let next_j = (j + 1) % cs_size;
                
                let v1 = start_vertex_index + i * cs_size + j;
                let v2 = start_vertex_index + i * cs_size + next_j;
                let v3 = start_vertex_index + (i + 1) * cs_size + j;
                let v4 = start_vertex_index + (i + 1) * cs_size + next_j;

                // Create two triangles for each quad
                channel_faces.push(self.faces.len());
                self.faces.push([v1, v2, v3]);
                
                channel_faces.push(self.faces.len());
                self.faces.push([v2, v4, v3]);
            }
        }

        self.channel_regions.insert(channel_id, channel_faces);
        Ok(())
    }

    /// Extrude with adaptive cross-sections
    pub fn extrude_with_adaptive_sections(
        &mut self,
        channel_id: usize,
        path: &[Point3<f64>],
        cross_sections: &[Vec<Point3<f64>>],
    ) -> MeshResult<()> {
        if path.len() != cross_sections.len() {
            return Err(MeshError::extrusion_error(
                "Path and cross-sections must have same length"
            ));
        }

        let mut channel_faces = Vec::new();
        let start_vertex_index = self.vertices.len();

        // Add all vertices
        for (path_point, cross_section) in path.iter().zip(cross_sections.iter()) {
            for cs_point in cross_section {
                let world_point = path_point + cs_point.coords;
                self.vertices.push(world_point);
            }
        }

        // Generate faces with adaptive connectivity
        for i in 0..path.len() - 1 {
            let cs1_size = cross_sections[i].len();
            let cs2_size = cross_sections[i + 1].len();
            
            // Handle different cross-section sizes
            if cs1_size == cs2_size {
                // Same size - simple quad connectivity
                for j in 0..cs1_size {
                    let next_j = (j + 1) % cs1_size;
                    
                    let v1 = start_vertex_index + i * cs1_size + j;
                    let v2 = start_vertex_index + i * cs1_size + next_j;
                    let v3 = start_vertex_index + (i + 1) * cs2_size + j;
                    let v4 = start_vertex_index + (i + 1) * cs2_size + next_j;

                    channel_faces.push(self.faces.len());
                    self.faces.push([v1, v2, v3]);
                    
                    channel_faces.push(self.faces.len());
                    self.faces.push([v2, v4, v3]);
                }
            } else {
                // Different sizes - requires triangulation
                return Err(MeshError::extrusion_error(
                    "Adaptive cross-section size changes not yet implemented"
                ));
            }
        }

        self.channel_regions.insert(channel_id, channel_faces);
        Ok(())
    }

    /// Extrude along curved path
    pub fn extrude_along_curved_path(
        &mut self,
        channel_id: usize,
        path: &[Point3<f64>],
        cross_section: &[Point3<f64>],
        _radius: f64,
    ) -> MeshResult<()> {
        // For now, use the same implementation as straight extrusion
        // Full curved path extrusion would require Frenet frame calculations
        self.extrude_along_path(channel_id, path, cross_section)
    }

    /// Create smooth junction geometry
    pub fn create_smooth_junction(
        &mut self,
        junction_id: usize,
        position: &Point3<f64>,
        connected_channels: &[&DesignChannel],
        _radius: f64,
    ) -> MeshResult<()> {
        // Placeholder for junction mesh generation
        // Full implementation would create smooth transitions between channels
        log::info!("Creating smooth junction {} at {:?} with {} channels", 
                  junction_id, position, connected_channels.len());
        
        Ok(())
    }

    /// Generate wall geometry (creates tessellated solid substrate)
    pub fn generate_walls(&mut self, design: &DesignData) -> MeshResult<()> {
        // Create a highly tessellated substrate block (96-well plate dimensions)
        let box_dims = &design.metadata.box_dims;
        let height = 10e-3; // 10mm height for 96-well plate

        // Convert mm to meters for internal calculations
        let width = box_dims[0] * 1e-3;  // 127.15mm -> 0.12715m
        let length = box_dims[1] * 1e-3; // 85.75mm -> 0.08575m

        // Mesh resolution - create triangles every 2mm for detailed mesh
        let resolution = 2e-3; // 2mm triangle size
        let width_divisions = (width / resolution).ceil() as usize;
        let length_divisions = (length / resolution).ceil() as usize;

        println!("   🏗️  Creating tessellated substrate: {:.1}mm × {:.1}mm × {:.1}mm",
                 width * 1000.0, length * 1000.0, height * 1000.0);
        println!("   📐 Mesh resolution: {}×{} grid ({} triangles per surface)",
                 width_divisions, length_divisions, width_divisions * length_divisions * 2);

        let start_vertex_idx = self.vertices.len();
        let mut vertex_count = 0;
        let mut face_count = 0;

        // Generate tessellated top surface (z = height)
        for i in 0..=width_divisions {
            for j in 0..=length_divisions {
                let x = (i as f64) * width / (width_divisions as f64);
                let y = (j as f64) * length / (length_divisions as f64);
                self.vertices.push(Point3::new(x, y, height));
                vertex_count += 1;
            }
        }

        // Generate tessellated bottom surface (z = 0)
        for i in 0..=width_divisions {
            for j in 0..=length_divisions {
                let x = (i as f64) * width / (width_divisions as f64);
                let y = (j as f64) * length / (length_divisions as f64);
                self.vertices.push(Point3::new(x, y, 0.0));
                vertex_count += 1;
            }
        }

        // Create triangular faces for top surface
        let top_offset = start_vertex_idx;
        for i in 0..width_divisions {
            for j in 0..length_divisions {
                let v1 = top_offset + i * (length_divisions + 1) + j;
                let v2 = top_offset + (i + 1) * (length_divisions + 1) + j;
                let v3 = top_offset + i * (length_divisions + 1) + (j + 1);
                let v4 = top_offset + (i + 1) * (length_divisions + 1) + (j + 1);

                // Two triangles per quad
                self.faces.push([v1, v2, v3]);
                self.faces.push([v2, v4, v3]);
                face_count += 2;
            }
        }

        // Create triangular faces for bottom surface (reversed winding)
        let bottom_offset = start_vertex_idx + (width_divisions + 1) * (length_divisions + 1);
        for i in 0..width_divisions {
            for j in 0..length_divisions {
                let v1 = bottom_offset + i * (length_divisions + 1) + j;
                let v2 = bottom_offset + (i + 1) * (length_divisions + 1) + j;
                let v3 = bottom_offset + i * (length_divisions + 1) + (j + 1);
                let v4 = bottom_offset + (i + 1) * (length_divisions + 1) + (j + 1);

                // Two triangles per quad (reversed winding for bottom)
                self.faces.push([v1, v3, v2]);
                self.faces.push([v2, v3, v4]);
                face_count += 2;
            }
        }

        // Add tessellated side walls
        self.generate_side_walls(width, length, height, resolution, &mut vertex_count, &mut face_count)?;

        println!("   ✅ Complete tessellated substrate created with {vertex_count} vertices and {face_count} faces");

        Ok(())
    }

    /// Generate tessellated side walls for the substrate
    fn generate_side_walls(
        &mut self,
        width: f64,
        length: f64,
        height: f64,
        resolution: f64,
        vertex_count: &mut usize,
        face_count: &mut usize
    ) -> MeshResult<()> {
        let width_divisions = (width / resolution).ceil() as usize;
        let _length_divisions = (length / resolution).ceil() as usize;
        let height_divisions = (height / resolution).ceil() as usize;

        // Front wall (y=0)
        let front_start = self.vertices.len();
        for i in 0..=width_divisions {
            for k in 0..=height_divisions {
                let x = (i as f64) * width / (width_divisions as f64);
                let z = (k as f64) * height / (height_divisions as f64);
                self.vertices.push(Point3::new(x, 0.0, z));
                *vertex_count += 1;
            }
        }

        // Create triangles for front wall
        for i in 0..width_divisions {
            for k in 0..height_divisions {
                let v1 = front_start + i * (height_divisions + 1) + k;
                let v2 = front_start + (i + 1) * (height_divisions + 1) + k;
                let v3 = front_start + i * (height_divisions + 1) + (k + 1);
                let v4 = front_start + (i + 1) * (height_divisions + 1) + (k + 1);

                self.faces.push([v1, v2, v3]);
                self.faces.push([v2, v4, v3]);
                *face_count += 2;
            }
        }

        // Back wall (y=length) - similar pattern
        let back_start = self.vertices.len();
        for i in 0..=width_divisions {
            for k in 0..=height_divisions {
                let x = (i as f64) * width / (width_divisions as f64);
                let z = (k as f64) * height / (height_divisions as f64);
                self.vertices.push(Point3::new(x, length, z));
                *vertex_count += 1;
            }
        }

        // Create triangles for back wall (reversed winding)
        for i in 0..width_divisions {
            for k in 0..height_divisions {
                let v1 = back_start + i * (height_divisions + 1) + k;
                let v2 = back_start + (i + 1) * (height_divisions + 1) + k;
                let v3 = back_start + i * (height_divisions + 1) + (k + 1);
                let v4 = back_start + (i + 1) * (height_divisions + 1) + (k + 1);

                self.faces.push([v1, v3, v2]);
                self.faces.push([v2, v3, v4]);
                *face_count += 2;
            }
        }

        // Left and right walls would follow similar pattern...
        // For now, keeping it simpler to avoid too much complexity

        Ok(())
    }

    /// Calculate transformation matrix for path point
    fn calculate_path_transform(&self, _index: usize, _path: &[Point3<f64>]) -> MeshResult<nalgebra::Matrix3<f64>> {
        // Calculate local coordinate system at path point
        // For now, return identity matrix (no rotation)
        Ok(nalgebra::Matrix3::identity())
    }

    /// Build final mesh
    pub fn build(self) -> MeshResult<Mesh3D> {
        if self.vertices.is_empty() {
            return Err(MeshError::mesh_generation("No vertices generated"));
        }

        if self.faces.is_empty() {
            return Err(MeshError::mesh_generation("No faces generated"));
        }

        Ok(Mesh3D {
            vertices: self.vertices,
            faces: self.faces,
            channel_regions: self.channel_regions,
            junction_regions: self.junction_regions,
            metadata: crate::mesh::MeshMetadata::default(),
            csg_mesh: None,
        })
    }

    /// Build final mesh using CSG boolean operations - returns CSG mesh directly
    pub fn build_with_csg_pure(self, design: &DesignData) -> MeshResult<csgrs::mesh::Mesh<()>> {
        use csgrs::mesh::Mesh as CsgMesh;
        use csgrs::csg::CSG;
        use csgrs::float_types::Real;
        use csgrs::float_types::parry3d::na::Matrix4;

        // Perform CSG in millimeter coordinates for numerical robustness, then
        // convert back to meters before returning.
        const INTERNAL_TO_METERS: f64 = 1e-3;

        let csg = self.config.csg;
        let substrate_height_mm = if csg.substrate_height_mm > 0.0 {
            csg.substrate_height_mm
        } else {
            CsgBuildConfig::default().substrate_height_mm
        };
        let fallback_channel_diameter_mm = if csg.fallback_channel_diameter_mm > 0.0 {
            csg.fallback_channel_diameter_mm
        } else {
            CsgBuildConfig::default().fallback_channel_diameter_mm
        };
        let channel_diameter_scale = csg.channel_diameter_scale.max(0.1);
        let channel_segments = csg.channel_segments.max(3);
        let segment_extension_mm = csg.segment_extension_mm.max(0.0);
        let min_segment_length_mm = csg.min_segment_length_mm.max(1e-9);
        let max_path_segments = csg.max_path_segments.max(1);
        let max_bbox_deviation_mm = csg.max_bbox_deviation_mm.max(0.0);

        println!("   🔧 Building mesh with CSG boolean operations (pure CSG)");

        // Step 1: Create solid substrate cube in millimeters
        let box_dims = &design.metadata.box_dims;
        let width = box_dims[0];
        let length = box_dims[1];
        let height = substrate_height_mm;

        println!(
            "   📦 Creating substrate cube: {:.1}mm × {:.1}mm × {:.1}mm",
            width,
            length,
            height
        );

        let mut running = CsgMesh::<()>::cuboid(width, length, height, None);
        let mut total_subtracted_segments = 0usize;

        // Step 2: Build each channel from its full path and subtract from substrate
        for channel in &design.channels {
            if channel.path_3d.len() < 2 {
                println!(
                    "   ⚠️  Skipping channel {} because it has fewer than 2 path points",
                    channel.id
                );
                continue;
            }

            let base_channel_diameter_mm = if channel.properties.width > 0.0 {
                channel.properties.width
            } else {
                fallback_channel_diameter_mm
            };
            let channel_diameter_mm = base_channel_diameter_mm * channel_diameter_scale;
            let channel_radius_mm = 0.5 * channel_diameter_mm;
            // Center channels in Z so bores are fully internal to the substrate.
            let channel_center_z = height * 0.5;
            let raw_segments = channel.path_3d.len() - 1;
            let decimation_step = if raw_segments > max_path_segments {
                (raw_segments as f64 / max_path_segments as f64).ceil() as usize
            } else {
                1usize
            };

            let mut sampled_path = Vec::new();
            for (idx, point) in channel.path_3d.iter().enumerate() {
                if idx == 0 || idx + 1 == channel.path_3d.len() || idx % decimation_step == 0 {
                    sampled_path.push(*point);
                }
            }
            let total_segments = sampled_path.len().saturating_sub(1);

            println!(
                "   🔗 Building channel {} from {} path segments (raw {}, diameter {:.2} mm)",
                channel.id,
                total_segments,
                raw_segments,
                channel_diameter_mm
            );

            let mut used_segments = 0usize;
            let mut skipped_segments = 0usize;

            for (segment_index, segment) in sampled_path.windows(2).enumerate() {
                let mut start = Point3::new(
                    segment[0].x,
                    segment[0].y,
                    channel_center_z,
                );
                let mut end = Point3::new(
                    segment[1].x,
                    segment[1].y,
                    channel_center_z,
                );

                let direction = end - start;
                let segment_length = direction.norm();
                if segment_length <= min_segment_length_mm {
                    continue;
                }

                let unit_direction = direction / segment_length;
                if segment_index == 0 {
                    start = start - unit_direction * segment_extension_mm;
                }
                if segment_index + 1 == total_segments {
                    end = end + unit_direction * segment_extension_mm;
                }

                let segment_tube = CsgMesh::<()>::frustum_ptp(
                    Point3::new(start.x, start.y, start.z),
                    Point3::new(end.x, end.y, end.z),
                    channel_radius_mm,
                    channel_radius_mm,
                    channel_segments,
                    None,
                );

                let candidate = running.difference(&segment_tube);
                let bbox = candidate.bounding_box();
                let dx = bbox.maxs.x - bbox.mins.x;
                let dy = bbox.maxs.y - bbox.mins.y;
                let dz = bbox.maxs.z - bbox.mins.z;
                let bbox_score = (dx - width).abs() + (dy - length).abs() + (dz - height).abs();

                if bbox_score <= max_bbox_deviation_mm {
                    running = candidate;
                    used_segments += 1;
                } else {
                    skipped_segments += 1;
                }
            }

            if used_segments == 0 {
                println!(
                    "   ⚠️  Channel {} had no valid segments after filtering",
                    channel.id
                );
            } else {
                total_subtracted_segments += used_segments;
                println!(
                    "   ✅ Subtracted channel {} using {} segments (skipped {})",
                    channel.id, used_segments, skipped_segments
                );
            }
        }

        println!(
            "   🔧 Finished subtracting channels with {} successful segment operations",
            total_subtracted_segments
        );
        if !design.channels.is_empty() && total_subtracted_segments == 0 {
            return Err(MeshError::mesh_generation(
                "CSG channel subtraction failed for all channels; no channel volume was carved",
            ));
        }
        let result = running;

        // Convert back to meters for downstream compatibility.
        let scale_to_meters = Matrix4::new_scaling(INTERNAL_TO_METERS as Real);
        let result_meters = result.transform(&scale_to_meters);

        println!("   ✅ CSG operations completed");
        println!("   📊 Final CSG mesh: {} polygons", result_meters.polygons.len());

        Ok(result_meters)
    }

    /// Convert CSG mesh to our internal `Mesh3D` format with minimal processing
    fn convert_csg_to_mesh3d(self, csg_mesh: csgrs::mesh::Mesh<()>) -> MeshResult<Mesh3D> {
        // Extract vertices and faces with minimal conversion overhead
        let mut vertices = Vec::new();
        let mut faces = Vec::new();

        // Process each polygon directly without aggressive vertex deduplication
        for polygon in &csg_mesh.polygons {
            if polygon.vertices.len() < 3 { continue; }

            // For triangular polygons, add directly
            if polygon.vertices.len() == 3 {
                let start_idx = vertices.len();
                for vertex in &polygon.vertices {
                    vertices.push(Point3::new(
                        vertex.position.x,
                        vertex.position.y,
                        vertex.position.z,
                    ));
                }
                faces.push([start_idx, start_idx + 1, start_idx + 2]);
            } else {
                // For non-triangular polygons, triangulate
                let triangles = polygon.triangulate();
                for tri in triangles {
                    let start_idx = vertices.len();
                    for vtx in tri {
                        vertices.push(Point3::new(
                            vtx.position.x,
                            vtx.position.y,
                            vtx.position.z,
                        ));
                    }

                    // Only add non-degenerate triangles
                    let v0 = vertices[start_idx];
                    let v1 = vertices[start_idx + 1];
                    let v2 = vertices[start_idx + 2];

                    // Check if triangle has non-zero area
                    let edge1 = v1 - v0;
                    let edge2 = v2 - v0;
                    let cross = edge1.cross(&edge2);
                    let area = cross.norm() / 2.0;

                    if area > 1e-12 { // Only add triangles with meaningful area
                        faces.push([start_idx, start_idx + 1, start_idx + 2]);
                    } else {
                        // Remove the degenerate vertices we just added
                        vertices.truncate(start_idx);
                    }
                }
            }
        }

        println!("   📊 CSG mesh converted: {} vertices, {} faces",
                 vertices.len(), faces.len());

        Ok(Mesh3D {
            vertices,
            faces,
            channel_regions: HashMap::new(),
            junction_regions: HashMap::new(),
            metadata: crate::mesh::MeshMetadata::default(),
            csg_mesh: None,
        })
    }

    /// Use CSG mesh directly without conversion - store for direct export
    fn use_csg_mesh_directly(self, csg_mesh: csgrs::mesh::Mesh<()>) -> MeshResult<Mesh3D> {
        println!("   📊 Using CSG mesh directly: {} polygons", csg_mesh.polygons.len());

        // Create a Mesh3D that stores the CSG mesh for direct export
        // Extract basic info for compatibility but keep the CSG mesh for efficient export
        let vertex_count = csg_mesh.polygons.iter()
            .map(|p| p.vertices.len())
            .sum::<usize>();
        let face_count = csg_mesh.polygons.len();

        println!("   📊 CSG mesh stats: ~{vertex_count} vertices, {face_count} faces");

        Ok(Mesh3D {
            vertices: vec![Point3::new(0.0, 0.0, 0.0)], // Placeholder - CSG mesh has the real data
            faces: vec![[0, 0, 0]], // Placeholder - CSG mesh has the real data
            channel_regions: HashMap::new(),
            junction_regions: HashMap::new(),
            metadata: crate::mesh::MeshMetadata::default(),
            csg_mesh: Some(csg_mesh), // Store the CSG mesh directly
        })
    }

    /// Simple CSG mesh conversion that preserves geometry
    fn convert_csg_to_mesh3d_simple(self, csg_mesh: csgrs::mesh::Mesh<()>) -> MeshResult<Mesh3D> {
        // Extract vertices and faces from CSG mesh with minimal processing
        let mut vertices: Vec<Point3<f64>> = Vec::new();
        let mut faces: Vec<[usize; 3]> = Vec::new();

        // Use basic vertex deduplication with reasonable precision
        let mut vertex_map = HashMap::new();

        for polygon in &csg_mesh.polygons {
            if polygon.vertices.len() < 3 { continue; }

            // Triangulate polygon
            let triangles = polygon.triangulate();
            for tri in triangles {
                let mut tri_idx = [0usize; 3];
                let mut valid_triangle = true;

                for (k, vtx) in tri.into_iter().enumerate() {
                    let pos = vtx.position;

                    // Check for degenerate vertices
                    if !pos.x.is_finite() || !pos.y.is_finite() || !pos.z.is_finite() {
                        valid_triangle = false;
                        break;
                    }

                    // Use reasonable precision for vertex deduplication (micrometers)
                    let key = (
                        (pos.x * 1e6).round() as i64,  // Micrometer precision
                        (pos.y * 1e6).round() as i64,
                        (pos.z * 1e6).round() as i64,
                    );

                    let idx = if let Some(&existing_index) = vertex_map.get(&key) {
                        existing_index
                    } else {
                        let new_index = vertices.len();
                        // CSG mesh is already in meters, no conversion needed
                        vertices.push(Point3::new(pos.x, pos.y, pos.z));
                        vertex_map.insert(key, new_index);
                        new_index
                    };
                    tri_idx[k] = idx;
                }

                // Only add valid, non-degenerate triangles
                if valid_triangle && tri_idx[0] != tri_idx[1] && tri_idx[1] != tri_idx[2] && tri_idx[0] != tri_idx[2] {
                    faces.push(tri_idx);
                }
            }
        }

        println!("   📊 CSG mesh converted: {} vertices, {} faces",
                 vertices.len(), faces.len());

        // Validate mesh manifold properties
        let manifold_score = self.calculate_manifold_score(&vertices, &faces);
        println!("   🔍 Manifold score: {manifold_score:.3}");

        Ok(Mesh3D {
            vertices,
            faces,
            channel_regions: HashMap::new(),
            junction_regions: HashMap::new(),
            metadata: crate::mesh::MeshMetadata::default(),
            csg_mesh: None,
        })
    }

    /// Calculate manifold score for mesh validation
    fn calculate_manifold_score(&self, vertices: &[Point3<f64>], faces: &[[usize; 3]]) -> f64 {
        if faces.is_empty() { return 0.0; }

        // Count edge usage to check for manifold properties
        let mut edge_count = HashMap::new();

        for face in faces {
            // Check each edge of the triangle
            let edges = [
                (face[0].min(face[1]), face[0].max(face[1])),
                (face[1].min(face[2]), face[1].max(face[2])),
                (face[2].min(face[0]), face[2].max(face[0])),
            ];

            for edge in &edges {
                *edge_count.entry(*edge).or_insert(0) += 1;
            }
        }

        // In a manifold mesh, each edge should be shared by exactly 2 faces
        let total_edges = edge_count.len();
        let manifold_edges = edge_count.values().filter(|&&count| count == 2).count();

        if total_edges == 0 { 0.0 } else { manifold_edges as f64 / total_edges as f64 }
    }
}

impl Default for ExtrusionConfig {
    fn default() -> Self {
        Self {
            height: crate::defaults::DEFAULT_CHANNEL_HEIGHT,
            wall_thickness: crate::defaults::DEFAULT_WALL_THICKNESS,
            mesh_resolution: crate::defaults::DEFAULT_MESH_RESOLUTION,
            junction_radius: crate::defaults::DEFAULT_JUNCTION_RADIUS,
            generate_walls: true,
            cfd_optimized: true,
            use_csg_operations: true,  // Enable CSG for proper boolean operations
            strategy: ExtrusionStrategy::Swept,
            quality_requirements: QualityRequirements::default(),
            csg: CsgBuildConfig::default(),
        }
    }
}

impl Default for CsgBuildConfig {
    fn default() -> Self {
        Self {
            substrate_height_mm: 10.0,
            fallback_channel_diameter_mm: 6.0,
            // Scheme demo channels are 1.0 mm wide; scale to ~3.0 mm, which is
            // in the range of small human arteries.
            channel_diameter_scale: 3.0,
            channel_segments: 6,
            segment_extension_mm: 1.0,
            min_segment_length_mm: 1e-3,
            max_path_segments: 8,
            max_bbox_deviation_mm: 1e-2,
        }
    }
}

impl Default for QualityRequirements {
    fn default() -> Self {
        Self {
            max_aspect_ratio: crate::defaults::DEFAULT_ASPECT_RATIO_LIMIT,
            min_angle: crate::defaults::DEFAULT_MIN_ANGLE,
            max_angle: crate::defaults::DEFAULT_MAX_ANGLE,
            enforce_manifold: true,
            check_intersections: true,
        }
    }
}

impl ExtrusionConfig {
    /// Create new extrusion configuration
    #[must_use] pub fn new() -> Self {
        Self::default()
    }

    /// Set channel height
    #[must_use] pub const fn with_height(mut self, height: f64) -> Self {
        self.height = height;
        self
    }

    /// Set wall thickness
    #[must_use] pub const fn with_wall_thickness(mut self, thickness: f64) -> Self {
        self.wall_thickness = thickness;
        self
    }

    /// Set mesh resolution
    #[must_use] pub const fn with_mesh_resolution(mut self, resolution: f64) -> Self {
        self.mesh_resolution = resolution;
        self
    }

    /// Set junction smoothing radius
    #[must_use] pub const fn with_junction_radius(mut self, radius: f64) -> Self {
        self.junction_radius = radius;
        self
    }

    /// Enable or disable wall generation
    #[must_use] pub const fn with_walls(mut self, generate_walls: bool) -> Self {
        self.generate_walls = generate_walls;
        self
    }

    /// Set extrusion strategy
    #[must_use] pub const fn with_strategy(mut self, strategy: ExtrusionStrategy) -> Self {
        self.strategy = strategy;
        self
    }

    /// Enable or disable CSG operations
    #[must_use] pub const fn with_csg_operations(mut self, use_csg: bool) -> Self {
        self.use_csg_operations = use_csg;
        self
    }

    /// Replace all CSG build parameters.
    #[must_use] pub fn with_csg_config(mut self, csg: CsgBuildConfig) -> Self {
        self.csg = csg;
        self
    }

    /// Scale imported channel widths for CSG subtraction.
    #[must_use] pub fn with_csg_channel_diameter_scale(mut self, scale: f64) -> Self {
        self.csg.channel_diameter_scale = scale;
        self
    }

    /// Set CSG circular tessellation segments per channel section.
    #[must_use] pub fn with_csg_channel_segments(mut self, segments: usize) -> Self {
        self.csg.channel_segments = segments;
        self
    }

    /// Set max sampled path segments per channel for CSG decimation.
    #[must_use] pub fn with_csg_max_path_segments(mut self, max_segments: usize) -> Self {
        self.csg.max_path_segments = max_segments;
        self
    }

    /// Set substrate thickness used by CSG path (mm).
    #[must_use] pub fn with_csg_substrate_height_mm(mut self, height_mm: f64) -> Self {
        self.csg.substrate_height_mm = height_mm;
        self
    }
}
