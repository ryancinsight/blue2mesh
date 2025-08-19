//! Scheme JSON import and data conversion
//!
//! This module provides robust parsing and validation of scheme package JSON
//! exports, converting them to internal data structures suitable for 3D mesh
//! generation with comprehensive error handling and validation.

use crate::error::{MeshError, MeshResult};
use nalgebra::{Point2, Point3};
use scheme::geometry::types::{ChannelSystem, Node, Channel, ChannelType as SchemeChannelType};
use serde::{Deserialize, Serialize};
use serde_json;
use chrono;
use std::collections::HashMap;
use std::fs;
use std::path::Path;

/// Imported design data from scheme package
#[derive(Debug, Clone)]
pub struct DesignData {
    /// Design metadata
    pub metadata: DesignMetadata,
    /// Network nodes (junctions, inlets, outlets)
    pub nodes: Vec<DesignNode>,
    /// Network channels connecting nodes
    pub channels: Vec<DesignChannel>,
    /// Overall design bounds
    pub bounds: DesignBounds,
    /// Design validation status
    pub validation_status: ValidationStatus,
}

/// Design metadata from scheme
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DesignMetadata {
    /// Design name
    pub name: String,
    /// Design description
    pub description: String,
    /// Design dimensions (width, height) in meters
    pub box_dims: [f64; 2],
    /// Creation timestamp
    pub created_at: Option<String>,
    /// Design version
    pub version: String,
    /// Additional metadata from scheme
    pub additional: HashMap<String, String>,
}

/// Design node with 3D positioning capability
#[derive(Debug, Clone, PartialEq)]
pub struct DesignNode {
    /// Node identifier
    pub id: usize,
    /// 2D position from scheme
    pub position_2d: Point2<f64>,
    /// 3D position (z-coordinate added during processing)
    pub position_3d: Point3<f64>,
    /// Node type classification
    pub node_type: NodeType,
    /// Node diameter for mesh generation
    pub diameter: f64,
    /// Additional node metadata
    pub metadata: HashMap<String, String>,
}

/// Design channel with 3D geometry information
#[derive(Debug, Clone)]
pub struct DesignChannel {
    /// Channel identifier
    pub id: usize,
    /// Source node ID
    pub from_node: usize,
    /// Target node ID
    pub to_node: usize,
    /// 2D path points from scheme
    pub path_2d: Vec<Point2<f64>>,
    /// 3D path points (z-coordinates added)
    pub path_3d: Vec<Point3<f64>>,
    /// Channel properties for mesh generation
    pub properties: ChannelMeshProperties,
    /// Channel type from scheme
    pub channel_type: ChannelType,
    /// Additional channel metadata
    pub metadata: HashMap<String, String>,
}

/// Node type classification for mesh generation
#[derive(Debug, Clone, PartialEq)]
pub enum NodeType {
    /// Fluid inlet
    Inlet,
    /// Fluid outlet
    Outlet,
    /// Internal junction
    Junction,
    /// Reservoir or dead end
    Reservoir,
}

/// Channel type for mesh generation
#[derive(Debug, Clone, PartialEq)]
pub enum ChannelType {
    /// Straight channel
    Straight,
    /// Serpentine (meandering) channel
    Serpentine { turns: usize },
    /// Arc channel
    Arc { radius: f64, angle: f64 },
    /// Frustum (tapered) channel
    Frustum { inlet_width: f64, outlet_width: f64 },
    /// Smooth straight channel
    SmoothStraight,
}

/// Channel properties for mesh generation
#[derive(Debug, Clone)]
pub struct ChannelMeshProperties {
    /// Channel width (m)
    pub width: f64,
    /// Channel height (m)
    pub height: f64,
    /// Channel length (m)
    pub length: f64,
    /// Wall thickness (m)
    pub wall_thickness: f64,
    /// Surface roughness (m)
    pub roughness: f64,
    /// Mesh resolution (m)
    pub mesh_resolution: f64,
    /// Whether channel requires special mesh treatment
    pub requires_refinement: bool,
}

/// Design bounds for mesh generation
#[derive(Debug, Clone)]
pub struct DesignBounds {
    /// Minimum x coordinate
    pub min_x: f64,
    /// Maximum x coordinate
    pub max_x: f64,
    /// Minimum y coordinate
    pub min_y: f64,
    /// Maximum y coordinate
    pub max_y: f64,
    /// Z-coordinate range for 3D extrusion
    pub z_range: (f64, f64),
}

/// Validation status for imported design
#[derive(Debug, Clone)]
pub struct ValidationStatus {
    /// Whether design passed validation
    pub valid: bool,
    /// Validation warnings
    pub warnings: Vec<String>,
    /// Validation errors
    pub errors: Vec<String>,
    /// Mesh generation recommendations
    pub recommendations: Vec<String>,
}

/// Scheme JSON importer with validation
pub struct SchemeImporter {
    /// Default channel height for 3D extrusion
    default_height: f64,
    /// Default wall thickness
    default_wall_thickness: f64,
    /// Default mesh resolution
    default_mesh_resolution: f64,
    /// Whether to validate imported data
    validate_on_import: bool,
}

impl SchemeImporter {
    /// Create new importer with default settings
    pub fn new() -> Self {
        Self {
            default_height: crate::defaults::DEFAULT_CHANNEL_HEIGHT,
            default_wall_thickness: crate::defaults::DEFAULT_WALL_THICKNESS,
            default_mesh_resolution: crate::defaults::DEFAULT_MESH_RESOLUTION,
            validate_on_import: true,
        }
    }

    /// Set default channel height
    pub fn with_default_height(mut self, height: f64) -> Self {
        self.default_height = height;
        self
    }

    /// Set default wall thickness
    pub fn with_default_wall_thickness(mut self, thickness: f64) -> Self {
        self.default_wall_thickness = thickness;
        self
    }

    /// Set default mesh resolution
    pub fn with_default_mesh_resolution(mut self, resolution: f64) -> Self {
        self.default_mesh_resolution = resolution;
        self
    }

    /// Enable or disable validation on import
    pub fn with_validation(mut self, validate: bool) -> Self {
        self.validate_on_import = validate;
        self
    }

    /// Import design from JSON file
    pub fn from_json_file<P: AsRef<Path>>(path: P) -> MeshResult<DesignData> {
        let importer = Self::new();
        importer.import_from_file(path)
    }

    /// Import design from JSON string
    pub fn from_json_string(json_str: &str) -> MeshResult<DesignData> {
        let importer = Self::new();
        importer.import_from_string(json_str)
    }

    /// Import from file with custom importer settings
    pub fn import_from_file<P: AsRef<Path>>(&self, path: P) -> MeshResult<DesignData> {
        let json_str = fs::read_to_string(path)
            .map_err(|e| MeshError::scheme_integration_error(
                format!("Failed to read scheme JSON file: {}", e)
            ))?;
        
        self.import_from_string(&json_str)
    }

    /// Import from JSON string with custom importer settings
    pub fn import_from_string(&self, json_str: &str) -> MeshResult<DesignData> {
        // Parse scheme JSON export
        let channel_system: ChannelSystem = serde_json::from_str(json_str)
            .map_err(|e| MeshError::scheme_integration_error(
                format!("Failed to parse scheme JSON: {}", e)
            ))?;

        self.convert_scheme_to_design(channel_system)
    }

    /// Convert scheme export to design data
    fn convert_scheme_to_design(&self, channel_system: ChannelSystem) -> MeshResult<DesignData> {
        // Convert metadata
        let metadata = DesignMetadata {
            name: "Millifluidic Design".to_string(),
            description: "Converted from scheme ChannelSystem".to_string(),
            box_dims: [channel_system.box_dims.0, channel_system.box_dims.1],
            created_at: Some(chrono::Utc::now().to_rfc3339()),
            version: "1.0".to_string(),
            additional: HashMap::new(),
        };

        // Convert nodes
        let mut nodes = Vec::new();
        for scheme_node in &channel_system.nodes {
            let design_node = self.convert_node(scheme_node, &channel_system)?;
            nodes.push(design_node);
        }

        // Convert channels
        let mut channels = Vec::new();
        for scheme_channel in &channel_system.channels {
            let design_channel = self.convert_channel(scheme_channel, &channel_system)?;
            channels.push(design_channel);
        }

        // Calculate design bounds
        let bounds = self.calculate_bounds(&nodes, &channels);

        // Validate design if enabled
        let validation_status = if self.validate_on_import {
            self.validate_design(&nodes, &channels, &bounds)?
        } else {
            ValidationStatus {
                valid: true,
                warnings: Vec::new(),
                errors: Vec::new(),
                recommendations: Vec::new(),
            }
        };

        Ok(DesignData {
            metadata,
            nodes,
            channels,
            bounds,
            validation_status,
        })
    }

    /// Convert scheme node to design node
    fn convert_node(&self, scheme_node: &Node, _channel_system: &ChannelSystem) -> MeshResult<DesignNode> {
        let position_2d = Point2::new(scheme_node.point.0, scheme_node.point.1);
        let position_3d = Point3::new(scheme_node.point.0, scheme_node.point.1, 0.0);

        // Determine node type based on connectivity (simplified for now)
        let node_type = NodeType::Junction; // Will be refined based on connectivity analysis

        Ok(DesignNode {
            id: scheme_node.id,
            position_2d,
            position_3d,
            node_type,
            diameter: 1e-3, // Default 1mm diameter
            metadata: HashMap::new(),
        })
    }

    /// Convert scheme channel to design channel
    fn convert_channel(&self, scheme_channel: &Channel, channel_system: &ChannelSystem) -> MeshResult<DesignChannel> {
        // Extract path from channel type and convert to 2D/3D paths
        let (path_2d, path_3d) = self.extract_channel_path(scheme_channel, channel_system)?;

        // Use channel dimensions from scheme
        let width = scheme_channel.width;
        let height = scheme_channel.height;

        let length = self.calculate_path_length_2d(&path_2d);

        let properties = ChannelMeshProperties {
            width,
            height,
            length,
            wall_thickness: self.default_wall_thickness,
            roughness: 1e-6, // Default smooth surface
            mesh_resolution: self.default_mesh_resolution,
            requires_refinement: false,
        };

        // Convert scheme channel type to design channel type
        let channel_type = self.convert_channel_type(&scheme_channel.channel_type)?;

        Ok(DesignChannel {
            id: scheme_channel.id,
            from_node: scheme_channel.from_node,
            to_node: scheme_channel.to_node,
            path_2d,
            path_3d,
            properties,
            channel_type,
            metadata: HashMap::new(), // Metadata is not serialized in scheme
        })
    }

    /// Extract path from scheme channel type
    fn extract_channel_path(&self, scheme_channel: &Channel, channel_system: &ChannelSystem) -> MeshResult<(Vec<Point2<f64>>, Vec<Point3<f64>>)> {
        let path_2d_tuples = match &scheme_channel.channel_type {
            SchemeChannelType::Straight => {
                // For straight channels, create path from node positions
                let from_node = &channel_system.nodes[scheme_channel.from_node];
                let to_node = &channel_system.nodes[scheme_channel.to_node];
                vec![from_node.point, to_node.point]
            },
            SchemeChannelType::SmoothStraight { path } => path.clone(),
            SchemeChannelType::Serpentine { path } => path.clone(),
            SchemeChannelType::Arc { path } => path.clone(),
            SchemeChannelType::Frustum { path, .. } => path.clone(),
        };

        // Convert tuple points to nalgebra Point2
        let path_2d: Vec<Point2<f64>> = path_2d_tuples.iter()
            .map(|p| Point2::new(p.0, p.1))
            .collect();

        // Convert 2D path to 3D path (z = 0 for now)
        let path_3d: Vec<Point3<f64>> = path_2d.iter()
            .map(|p| Point3::new(p.x, p.y, 0.0))
            .collect();

        Ok((path_2d, path_3d))
    }

    /// Convert scheme channel type to design channel type
    fn convert_channel_type(&self, scheme_type: &SchemeChannelType) -> MeshResult<ChannelType> {
        match scheme_type {
            SchemeChannelType::Straight => Ok(ChannelType::Straight),
            SchemeChannelType::SmoothStraight { .. } => Ok(ChannelType::SmoothStraight),
            SchemeChannelType::Serpentine { path } => {
                let turns = (path.len().saturating_sub(2)) / 2; // Estimate turns from path complexity
                Ok(ChannelType::Serpentine { turns })
            },
            SchemeChannelType::Arc { .. } => {
                // Default arc parameters - could be extracted from path analysis
                Ok(ChannelType::Arc { radius: 1e-3, angle: std::f64::consts::PI / 2.0 })
            },
            SchemeChannelType::Frustum { inlet_width, outlet_width, .. } => {
                Ok(ChannelType::Frustum {
                    inlet_width: *inlet_width,
                    outlet_width: *outlet_width
                })
            },
        }
    }

    /// Calculate path length from 2D points
    fn calculate_path_length_2d(&self, path: &[Point2<f64>]) -> f64 {
        if path.len() < 2 {
            return 0.0;
        }

        path.windows(2)
            .map(|window| {
                let dx = window[1].x - window[0].x;
                let dy = window[1].y - window[0].y;
                (dx * dx + dy * dy).sqrt()
            })
            .sum()
    }

    /// Parse channel type from metadata
    fn parse_channel_type(&self, metadata: &HashMap<String, String>) -> MeshResult<ChannelType> {
        let type_str = metadata.get("channel_type")
            .unwrap_or(&"straight".to_string())
            .to_lowercase();

        match type_str.as_str() {
            "straight" => Ok(ChannelType::Straight),
            "smoothstraight" => Ok(ChannelType::SmoothStraight),
            "serpentine" => {
                let turns = metadata.get("turns")
                    .and_then(|t| t.parse::<usize>().ok())
                    .unwrap_or(1);
                Ok(ChannelType::Serpentine { turns })
            }
            "arc" => {
                let radius = metadata.get("radius")
                    .and_then(|r| r.parse::<f64>().ok())
                    .unwrap_or(1e-3);
                let angle = metadata.get("angle")
                    .and_then(|a| a.parse::<f64>().ok())
                    .unwrap_or(std::f64::consts::PI / 2.0);
                Ok(ChannelType::Arc { radius, angle })
            }
            "frustum" => {
                let inlet_width = metadata.get("inlet_width")
                    .and_then(|w| w.parse::<f64>().ok())
                    .unwrap_or(100e-6);
                let outlet_width = metadata.get("outlet_width")
                    .and_then(|w| w.parse::<f64>().ok())
                    .unwrap_or(50e-6);
                Ok(ChannelType::Frustum { inlet_width, outlet_width })
            }
            _ => {
                log::warn!("Unknown channel type '{}', using Straight", type_str);
                Ok(ChannelType::Straight)
            }
        }
    }

    /// Calculate design bounds
    fn calculate_bounds(&self, nodes: &[DesignNode], channels: &[DesignChannel]) -> DesignBounds {
        let mut min_x = f64::INFINITY;
        let mut max_x = f64::NEG_INFINITY;
        let mut min_y = f64::INFINITY;
        let mut max_y = f64::NEG_INFINITY;

        // Check node positions
        for node in nodes {
            min_x = min_x.min(node.position_2d.x);
            max_x = max_x.max(node.position_2d.x);
            min_y = min_y.min(node.position_2d.y);
            max_y = max_y.max(node.position_2d.y);
        }

        // Check channel paths
        for channel in channels {
            for point in &channel.path_2d {
                min_x = min_x.min(point.x);
                max_x = max_x.max(point.x);
                min_y = min_y.min(point.y);
                max_y = max_y.max(point.y);
            }
        }

        // Add margin for wall thickness
        let margin = self.default_wall_thickness * 2.0;
        
        DesignBounds {
            min_x: min_x - margin,
            max_x: max_x + margin,
            min_y: min_y - margin,
            max_y: max_y + margin,
            z_range: (0.0, self.default_height),
        }
    }

    /// Validate imported design data
    fn validate_design(
        &self,
        nodes: &[DesignNode],
        channels: &[DesignChannel],
        bounds: &DesignBounds,
    ) -> MeshResult<ValidationStatus> {
        let mut warnings = Vec::new();
        let mut errors = Vec::new();
        let mut recommendations = Vec::new();

        // Check for minimum required elements
        if nodes.is_empty() {
            errors.push("Design must contain at least one node".to_string());
        }

        if channels.is_empty() {
            errors.push("Design must contain at least one channel".to_string());
        }

        // Check for reasonable dimensions
        let design_width = bounds.max_x - bounds.min_x;
        let design_height = bounds.max_y - bounds.min_y;

        if design_width < 1e-6 || design_height < 1e-6 {
            errors.push("Design dimensions are too small for mesh generation".to_string());
        }

        if design_width > 0.1 || design_height > 0.1 {
            warnings.push("Large design dimensions may require coarse mesh resolution".to_string());
        }

        // Check channel properties
        for channel in channels {
            if channel.properties.width < 1e-6 {
                warnings.push(format!("Channel {} has very small width: {:.2e} m", 
                                    channel.id, channel.properties.width));
            }

            if channel.properties.mesh_resolution > channel.properties.width / 5.0 {
                recommendations.push(format!(
                    "Channel {} mesh resolution should be refined for better quality",
                    channel.id
                ));
            }
        }

        // Check connectivity
        let connectivity_issues = self.check_connectivity(nodes, channels);
        errors.extend(connectivity_issues);

        let valid = errors.is_empty();

        Ok(ValidationStatus {
            valid,
            warnings,
            errors,
            recommendations,
        })
    }

    /// Check network connectivity
    fn check_connectivity(&self, nodes: &[DesignNode], channels: &[DesignChannel]) -> Vec<String> {
        let mut errors = Vec::new();
        let node_ids: std::collections::HashSet<usize> = nodes.iter().map(|n| n.id).collect();

        // Check that all channel endpoints reference valid nodes
        for channel in channels {
            if !node_ids.contains(&channel.from_node) {
                errors.push(format!("Channel {} references invalid source node {}", 
                                  channel.id, channel.from_node));
            }
            
            if !node_ids.contains(&channel.to_node) {
                errors.push(format!("Channel {} references invalid target node {}", 
                                  channel.id, channel.to_node));
            }
        }

        errors
    }
}

impl Default for SchemeImporter {
    fn default() -> Self {
        Self::new()
    }
}

impl DesignData {
    /// Get node by ID
    pub fn get_node(&self, node_id: usize) -> Option<&DesignNode> {
        self.nodes.iter().find(|n| n.id == node_id)
    }

    /// Get channel by ID
    pub fn get_channel(&self, channel_id: usize) -> Option<&DesignChannel> {
        self.channels.iter().find(|c| c.id == channel_id)
    }

    /// Get all inlet nodes
    pub fn inlet_nodes(&self) -> Vec<&DesignNode> {
        self.nodes.iter()
            .filter(|n| matches!(n.node_type, NodeType::Inlet))
            .collect()
    }

    /// Get all outlet nodes
    pub fn outlet_nodes(&self) -> Vec<&DesignNode> {
        self.nodes.iter()
            .filter(|n| matches!(n.node_type, NodeType::Outlet))
            .collect()
    }

    /// Get channels connected to a node
    pub fn channels_for_node(&self, node_id: usize) -> Vec<&DesignChannel> {
        self.channels.iter()
            .filter(|c| c.from_node == node_id || c.to_node == node_id)
            .collect()
    }

    /// Calculate total design volume
    pub fn total_volume(&self) -> f64 {
        self.channels.iter()
            .map(|c| c.properties.width * c.properties.height * c.properties.length)
            .sum()
    }

    /// Calculate total surface area
    pub fn total_surface_area(&self) -> f64 {
        self.channels.iter()
            .map(|c| {
                let perimeter = 2.0 * (c.properties.width + c.properties.height);
                perimeter * c.properties.length
            })
            .sum()
    }
}

impl ChannelMeshProperties {
    /// Create default properties
    pub fn default_properties() -> Self {
        Self {
            width: 100e-6,
            height: crate::defaults::DEFAULT_CHANNEL_HEIGHT,
            length: 1e-3,
            wall_thickness: crate::defaults::DEFAULT_WALL_THICKNESS,
            roughness: 1e-6,
            mesh_resolution: crate::defaults::DEFAULT_MESH_RESOLUTION,
            requires_refinement: false,
        }
    }

    /// Calculate hydraulic diameter
    pub fn hydraulic_diameter(&self) -> f64 {
        4.0 * (self.width * self.height) / (2.0 * (self.width + self.height))
    }

    /// Calculate aspect ratio
    pub fn aspect_ratio(&self) -> f64 {
        self.width.max(self.height) / self.width.min(self.height)
    }

    /// Estimate mesh element count
    pub fn estimated_element_count(&self) -> usize {
        let elements_per_length = (self.length / self.mesh_resolution).ceil() as usize;
        let elements_per_width = (self.width / self.mesh_resolution).ceil() as usize;
        let elements_per_height = (self.height / self.mesh_resolution).ceil() as usize;
        
        elements_per_length * elements_per_width * elements_per_height
    }
}
