//! Mesh quality validation and improvement
//!
//! This module provides comprehensive mesh quality analysis and improvement
//! algorithms to ensure CFD-ready meshes with proper geometric properties.

use crate::error::MeshResult;
use crate::mesh::Mesh3D;
use nalgebra::Point3;
use serde::{Deserialize, Serialize};

/// Mesh quality controller
pub struct QualityController {
    /// Quality requirements
    requirements: QualityRequirements,
    /// Whether to auto-fix quality issues
    auto_fix: bool,
}

/// Quality requirements for mesh validation
#[derive(Debug, Clone)]
pub struct QualityRequirements {
    /// Minimum acceptable quality score (0-1)
    pub min_quality_score: f64,
    /// Maximum aspect ratio
    pub max_aspect_ratio: f64,
    /// Minimum angle (degrees)
    pub min_angle: f64,
    /// Maximum angle (degrees)
    pub max_angle: f64,
    /// Maximum skewness
    pub max_skewness: f64,
    /// Whether mesh must be manifold
    pub require_manifold: bool,
    /// Whether to check for self-intersections
    pub check_intersections: bool,
}

/// Comprehensive mesh quality metrics
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QualityMetrics {
    /// Overall quality score (0-1, higher is better)
    pub overall_score: f64,
    /// Individual metric scores
    pub aspect_ratio_score: f64,
    pub angle_score: f64,
    pub skewness_score: f64,
    pub manifold_score: f64,
    /// Detailed statistics
    pub statistics: QualityStatistics,
    /// Quality issues found
    pub issues: Vec<QualityIssue>,
    /// Recommendations for improvement
    pub recommendations: Vec<String>,
}

/// Detailed quality statistics
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QualityStatistics {
    /// Total number of elements
    pub total_elements: usize,
    /// Number of high-quality elements
    pub high_quality_elements: usize,
    /// Number of poor-quality elements
    pub poor_quality_elements: usize,
    /// Aspect ratio statistics
    pub aspect_ratio: StatisticsSummary,
    /// Angle statistics
    pub min_angles: StatisticsSummary,
    pub max_angles: StatisticsSummary,
    /// Skewness statistics
    pub skewness: StatisticsSummary,
    /// Volume statistics
    pub element_volumes: StatisticsSummary,
}

/// Statistical summary for quality metrics
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StatisticsSummary {
    pub min: f64,
    pub max: f64,
    pub mean: f64,
    pub std_dev: f64,
    pub percentile_95: f64,
    pub percentile_99: f64,
}

/// Quality issue identification
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QualityIssue {
    /// Issue type
    pub issue_type: QualityIssueType,
    /// Severity level
    pub severity: IssueSeverity,
    /// Element indices affected
    pub affected_elements: Vec<usize>,
    /// Description of the issue
    pub description: String,
    /// Suggested fix
    pub suggested_fix: String,
}

/// Types of quality issues
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum QualityIssueType {
    HighAspectRatio,
    SmallAngle,
    LargeAngle,
    HighSkewness,
    DegenerateElement,
    NonManifold,
    SelfIntersection,
    NegativeVolume,
}

/// Severity levels for quality issues
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum IssueSeverity {
    Info,
    Warning,
    Error,
    Critical,
}

impl QualityController {
    /// Create new quality controller with default requirements
    #[must_use] pub fn new() -> Self {
        Self {
            requirements: QualityRequirements::default(),
            auto_fix: false,
        }
    }

    /// Create quality controller for CFD applications
    #[must_use] pub const fn cfd_ready() -> Self {
        Self {
            requirements: QualityRequirements::cfd_requirements(),
            auto_fix: true,
        }
    }

    /// Create quality controller for manufacturing
    #[must_use] pub const fn manufacturing_ready() -> Self {
        Self {
            requirements: QualityRequirements::manufacturing_requirements(),
            auto_fix: true,
        }
    }

    /// Set quality requirements
    #[must_use] pub const fn with_requirements(mut self, requirements: QualityRequirements) -> Self {
        self.requirements = requirements;
        self
    }

    /// Enable auto-fix of quality issues
    #[must_use] pub const fn with_auto_fix(mut self, auto_fix: bool) -> Self {
        self.auto_fix = auto_fix;
        self
    }

    /// Validate mesh quality
    pub fn validate_mesh(&self, mesh: &Mesh3D) -> MeshResult<QualityMetrics> {
        let mut issues = Vec::new();
        let mut recommendations = Vec::new();

        // Calculate aspect ratios
        let aspect_ratios = self.calculate_aspect_ratios(mesh)?;
        let aspect_ratio_score = self.score_aspect_ratios(&aspect_ratios);
        
        // Calculate angles
        let (min_angles, max_angles) = self.calculate_angles(mesh)?;
        let angle_score = self.score_angles(&min_angles, &max_angles);
        
        // Calculate skewness
        let skewness_values = self.calculate_skewness(mesh)?;
        let skewness_score = self.score_skewness(&skewness_values);
        
        // Check manifold property with detailed score
        let manifold_score = self.calculate_manifold_score(&mesh.vertices, &mesh.faces);
        
        // Find quality issues
        self.find_aspect_ratio_issues(&aspect_ratios, &mut issues);
        self.find_angle_issues(&min_angles, &max_angles, &mut issues);
        self.find_skewness_issues(&skewness_values, &mut issues);

        // Generate recommendations
        if aspect_ratio_score < 0.8 {
            recommendations.push("Consider mesh refinement to improve aspect ratios".to_string());
        }
        if angle_score < 0.8 {
            recommendations.push("Consider smoothing operations to improve angles".to_string());
        }

        // Calculate overall score
        let overall_score = (aspect_ratio_score + angle_score + skewness_score + manifold_score) / 4.0;

        // Generate statistics
        let statistics = QualityStatistics {
            total_elements: mesh.faces.len(),
            high_quality_elements: issues.iter().filter(|i| matches!(i.severity, IssueSeverity::Info)).count(),
            poor_quality_elements: issues.iter().filter(|i| !matches!(i.severity, IssueSeverity::Info)).count(),
            aspect_ratio: self.calculate_statistics(&aspect_ratios),
            min_angles: self.calculate_statistics(&min_angles),
            max_angles: self.calculate_statistics(&max_angles),
            skewness: self.calculate_statistics(&skewness_values),
            element_volumes: self.calculate_statistics(&self.calculate_element_volumes(mesh)?),
        };

        Ok(QualityMetrics {
            overall_score,
            aspect_ratio_score,
            angle_score,
            skewness_score,
            manifold_score,
            statistics,
            issues,
            recommendations,
        })
    }

    /// Calculate aspect ratios for all elements
    fn calculate_aspect_ratios(&self, mesh: &Mesh3D) -> MeshResult<Vec<f64>> {
        let mut aspect_ratios = Vec::new();

        for face in &mesh.faces {
            let v1 = mesh.vertices[face[0]];
            let v2 = mesh.vertices[face[1]];
            let v3 = mesh.vertices[face[2]];

            let edge1 = (v2 - v1).norm();
            let edge2 = (v3 - v2).norm();
            let edge3 = (v1 - v3).norm();

            let max_edge = edge1.max(edge2).max(edge3);
            let min_edge = edge1.min(edge2).min(edge3);

            let aspect_ratio = if min_edge > 1e-12 {
                max_edge / min_edge
            } else {
                f64::INFINITY
            };

            aspect_ratios.push(aspect_ratio);
        }

        Ok(aspect_ratios)
    }

    /// Calculate angles for all elements
    fn calculate_angles(&self, mesh: &Mesh3D) -> MeshResult<(Vec<f64>, Vec<f64>)> {
        let mut min_angles = Vec::new();
        let mut max_angles = Vec::new();

        for face in &mesh.faces {
            let v1 = mesh.vertices[face[0]];
            let v2 = mesh.vertices[face[1]];
            let v3 = mesh.vertices[face[2]];

            let angles = self.calculate_triangle_angles(v1, v2, v3);
            min_angles.push(angles.iter().fold(f64::INFINITY, |a, &b| a.min(b)));
            max_angles.push(angles.iter().fold(0.0_f64, |a, &b| a.max(b)));
        }

        Ok((min_angles, max_angles))
    }

    /// Calculate triangle angles in degrees
    fn calculate_triangle_angles(&self, v1: Point3<f64>, v2: Point3<f64>, v3: Point3<f64>) -> [f64; 3] {
        let edge1 = (v2 - v1).normalize();
        let edge2 = (v3 - v2).normalize();
        let edge3 = (v1 - v3).normalize();

        let angle1 = (-edge1.dot(&edge3)).acos().to_degrees();
        let angle2 = (-edge2.dot(&edge1)).acos().to_degrees();
        let angle3 = (-edge3.dot(&edge2)).acos().to_degrees();

        [angle1, angle2, angle3]
    }

    /// Calculate skewness for all elements
    fn calculate_skewness(&self, mesh: &Mesh3D) -> MeshResult<Vec<f64>> {
        let mut skewness_values = Vec::new();

        for face in &mesh.faces {
            let v1 = mesh.vertices[face[0]];
            let v2 = mesh.vertices[face[1]];
            let v3 = mesh.vertices[face[2]];

            // Calculate skewness as deviation from equilateral triangle
            let edge1 = (v2 - v1).norm();
            let edge2 = (v3 - v2).norm();
            let edge3 = (v1 - v3).norm();

            let avg_edge = (edge1 + edge2 + edge3) / 3.0;
            let skewness = if avg_edge > 1e-12 {
                ((edge1 - avg_edge).abs() + (edge2 - avg_edge).abs() + (edge3 - avg_edge).abs()) / (3.0 * avg_edge)
            } else {
                f64::INFINITY
            };

            skewness_values.push(skewness);
        }

        Ok(skewness_values)
    }

    /// Calculate element volumes
    fn calculate_element_volumes(&self, mesh: &Mesh3D) -> MeshResult<Vec<f64>> {
        let mut volumes = Vec::new();

        for face in &mesh.faces {
            let v1 = mesh.vertices[face[0]];
            let v2 = mesh.vertices[face[1]];
            let v3 = mesh.vertices[face[2]];

            // Calculate triangle area (volume for 2D elements)
            let edge1 = v2 - v1;
            let edge2 = v3 - v1;
            let area = edge1.cross(&edge2).norm() / 2.0;
            volumes.push(area);
        }

        Ok(volumes)
    }

    /// Check if mesh is manifold
    fn is_manifold(&self, mesh: &Mesh3D) -> MeshResult<bool> {
        // Calculate manifold score and return true if it's above threshold
        let manifold_score = self.calculate_manifold_score(&mesh.vertices, &mesh.faces);
        Ok(manifold_score > 0.8) // Consider manifold if 80% of edges are properly shared
    }

    /// Calculate manifold score for mesh validation
    fn calculate_manifold_score(&self, vertices: &[Point3<f64>], faces: &[[usize; 3]]) -> f64 {
        if faces.is_empty() { return 0.0; }

        // Count edge usage to check for manifold properties
        let mut edge_count = std::collections::HashMap::new();

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

    /// Score aspect ratios
    fn score_aspect_ratios(&self, aspect_ratios: &[f64]) -> f64 {
        let good_elements = aspect_ratios.iter()
            .filter(|&&ar| ar <= self.requirements.max_aspect_ratio)
            .count();
        
        good_elements as f64 / aspect_ratios.len() as f64
    }

    /// Score angles
    fn score_angles(&self, min_angles: &[f64], max_angles: &[f64]) -> f64 {
        let good_elements = min_angles.iter().zip(max_angles.iter())
            .filter(|(&min_ang, &max_ang)| {
                min_ang >= self.requirements.min_angle && max_ang <= self.requirements.max_angle
            })
            .count();
        
        good_elements as f64 / min_angles.len() as f64
    }

    /// Score skewness
    fn score_skewness(&self, skewness_values: &[f64]) -> f64 {
        let good_elements = skewness_values.iter()
            .filter(|&&skew| skew <= self.requirements.max_skewness)
            .count();
        
        good_elements as f64 / skewness_values.len() as f64
    }

    /// Find aspect ratio issues
    fn find_aspect_ratio_issues(&self, aspect_ratios: &[f64], issues: &mut Vec<QualityIssue>) {
        let bad_elements: Vec<usize> = aspect_ratios.iter()
            .enumerate()
            .filter(|(_, &ar)| ar > self.requirements.max_aspect_ratio)
            .map(|(i, _)| i)
            .collect();

        if !bad_elements.is_empty() {
            issues.push(QualityIssue {
                issue_type: crate::error::specialized::QualityValidationError::AspectRatioExceeded { 
                    ratio: aspect_ratios.iter().fold(0.0, |a, &b| a.max(b)), 
                    limit: self.requirements.max_aspect_ratio 
                }.into(),
                severity: if bad_elements.len() > aspect_ratios.len() / 10 {
                    IssueSeverity::Error
                } else {
                    IssueSeverity::Warning
                },
                affected_elements: bad_elements,
                description: "High aspect ratio elements found".to_string(),
                suggested_fix: "Refine mesh or adjust geometry".to_string(),
            });
        }
    }

    /// Find angle issues
    fn find_angle_issues(&self, min_angles: &[f64], _max_angles: &[f64], issues: &mut Vec<QualityIssue>) {
        let bad_min_elements: Vec<usize> = min_angles.iter()
            .enumerate()
            .filter(|(_, &ang)| ang < self.requirements.min_angle)
            .map(|(i, _)| i)
            .collect();

        if !bad_min_elements.is_empty() {
            issues.push(QualityIssue {
                issue_type: QualityIssueType::SmallAngle,
                severity: IssueSeverity::Warning,
                affected_elements: bad_min_elements,
                description: "Elements with small angles found".to_string(),
                suggested_fix: "Apply smoothing or remeshing".to_string(),
            });
        }
    }

    /// Find skewness issues
    fn find_skewness_issues(&self, skewness_values: &[f64], issues: &mut Vec<QualityIssue>) {
        let bad_elements: Vec<usize> = skewness_values.iter()
            .enumerate()
            .filter(|(_, &skew)| skew > self.requirements.max_skewness)
            .map(|(i, _)| i)
            .collect();

        if !bad_elements.is_empty() {
            issues.push(QualityIssue {
                issue_type: QualityIssueType::HighSkewness,
                severity: IssueSeverity::Warning,
                affected_elements: bad_elements,
                description: "Highly skewed elements found".to_string(),
                suggested_fix: "Apply mesh smoothing".to_string(),
            });
        }
    }

    /// Calculate statistics summary
    fn calculate_statistics(&self, values: &[f64]) -> StatisticsSummary {
        if values.is_empty() {
            return StatisticsSummary {
                min: 0.0, max: 0.0, mean: 0.0, std_dev: 0.0,
                percentile_95: 0.0, percentile_99: 0.0,
            };
        }

        let mut sorted_values = values.to_vec();
        sorted_values.sort_by(|a, b| a.partial_cmp(b).unwrap());

        let min = sorted_values[0];
        let max = sorted_values[sorted_values.len() - 1];
        let mean = values.iter().sum::<f64>() / values.len() as f64;
        
        let variance = values.iter()
            .map(|x| (x - mean).powi(2))
            .sum::<f64>() / values.len() as f64;
        let std_dev = variance.sqrt();

        let percentile_95 = sorted_values[(0.95 * sorted_values.len() as f64) as usize];
        let percentile_99 = sorted_values[(0.99 * sorted_values.len() as f64) as usize];

        StatisticsSummary {
            min, max, mean, std_dev, percentile_95, percentile_99
        }
    }
}

impl Default for QualityController {
    fn default() -> Self {
        Self::new()
    }
}

impl Default for QualityRequirements {
    fn default() -> Self {
        Self {
            min_quality_score: 0.7,
            max_aspect_ratio: crate::defaults::DEFAULT_ASPECT_RATIO_LIMIT,
            min_angle: crate::defaults::DEFAULT_MIN_ANGLE,
            max_angle: crate::defaults::DEFAULT_MAX_ANGLE,
            max_skewness: 0.8,
            require_manifold: true,
            check_intersections: true,
        }
    }
}

impl QualityRequirements {
    /// Requirements for CFD applications
    #[must_use] pub const fn cfd_requirements() -> Self {
        Self {
            min_quality_score: 0.8,
            max_aspect_ratio: 5.0,
            min_angle: 30.0,
            max_angle: 120.0,
            max_skewness: 0.6,
            require_manifold: true,
            check_intersections: true,
        }
    }

    /// Requirements for manufacturing applications
    #[must_use] pub const fn manufacturing_requirements() -> Self {
        Self {
            min_quality_score: 0.9,
            max_aspect_ratio: 3.0,
            min_angle: 45.0,
            max_angle: 135.0,
            max_skewness: 0.4,
            require_manifold: true,
            check_intersections: true,
        }
    }
}

/// Convert quality issue types
impl From<crate::error::specialized::QualityValidationError> for QualityIssueType {
    fn from(error: crate::error::specialized::QualityValidationError) -> Self {
        match error {
            crate::error::specialized::QualityValidationError::AspectRatioExceeded { .. } => {
                Self::HighAspectRatio
            }
            crate::error::specialized::QualityValidationError::MinAngleBelowThreshold { .. } => {
                Self::SmallAngle
            }
            crate::error::specialized::QualityValidationError::MaxAngleExceedsThreshold { .. } => {
                Self::LargeAngle
            }
            crate::error::specialized::QualityValidationError::DegenerateElements { .. } => {
                Self::DegenerateElement
            }
            crate::error::specialized::QualityValidationError::NonManifold { .. } => {
                Self::NonManifold
            }
            crate::error::specialized::QualityValidationError::SelfIntersections { .. } => {
                Self::SelfIntersection
            }
        }
    }
}
