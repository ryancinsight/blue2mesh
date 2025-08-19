# Blue2Mesh - 3D Mesh Generation for Millifluidic Devices

⚠️ **EXPERIMENTAL - NOT PRODUCTION READY** ⚠️

Blue2Mesh is an experimental Rust library for converting 2D millifluidic designs from the [scheme](https://github.com/ryancinsight/scheme) package into 3D meshes for CFD simulation and prototyping. This crate is currently in early development and is **not suitable for production use or manufacturing**.

**Key Dependencies:**

- [scheme](https://github.com/ryancinsight/scheme) - 2D millifluidic design patterns
- [csgrs](https://github.com/timschmidt/csgrs/tree/main/src) - 3D constructive solid geometry operations

## Features

### 🎯 **Core Capabilities**

- **Automated 3D Extrusion**: Convert 2D channel designs to 3D volumes
- **Channel-Specific Algorithms**: Optimized mesh generation for different channel types
- **Junction Optimization**: Smooth transitions at channel intersections
- **Quality Control**: Comprehensive mesh quality validation and improvement
- **Multi-Format Export**: CFD-ready (VTK, OpenFOAM) and manufacturing (STL) formats

### 🔬 **CFD Integration**

- **Boundary Layer Generation**: Automatic boundary layer mesh for accurate CFD
- **Region Marking**: Proper boundary condition assignment for CFD solvers
- **Quality Validation**: CFD-specific mesh quality requirements
- **OxiCFD Integration**: Seamless workflow with CFD analysis

### ⚠️ **Experimental Features (Not Production Ready)**

- **Manifold Validation**: Ensures printable geometry (experimental)
- **Wall Thickness Control**: Proper wall generation for structural integrity (experimental)
- **Multi-Scale Support**: From micrometers to millimeters (experimental)
- **Format Compatibility**: STL, OBJ, PLY export for various manufacturing processes (experimental)

## ⚠️ Important Notice

**This crate is experimental and not ready for production use.** It is intended for:

- Research and development
- Prototyping and concept validation
- Educational purposes
- Experimental mesh generation

**Do NOT use for:**

- Production manufacturing
- Critical applications
- Commercial products
- Safety-critical systems

## Quick Start

### Basic Usage (Experimental)

```rust
use blue2mesh::prelude::*;

// Import 2D design from scheme
let design = SchemeImporter::from_json_file("design.json")?;

// Generate 3D mesh
let mesh = MeshGenerator::new()
    .with_quality_level(QualityLevel::HighQuality)
    .generate_from_design(&design)?;

// Export for CFD
CfdExporter::new().export_vtk(&mesh, "device.vtk")?;

// Export for manufacturing
ManufacturingExporter::new().export_stl(&mesh, "device.stl")?;
```

### Complete Workflow

```rust
use blue2mesh::prelude::*;

// Configure complete pipeline
let config = Blue2MeshConfig {
    quality_level: QualityLevel::HighQuality,
    strategy: MeshStrategy::Hybrid,
    channel_height: 100e-6,  // 100 μm
    wall_thickness: 50e-6,   // 50 μm
    mesh_resolution: 10e-6,  // 10 μm
    optimize_junctions: true,
    validate_quality: true,
    export_formats: vec![ExportFormat::VTK, ExportFormat::STL],
};

// Process design through complete pipeline
let pipeline = Blue2MeshPipeline::new(config);
let output_files = pipeline.process_scheme_to_mesh(
    "design.json",
    "output/"
)?;
```

## Architecture

### Module Structure

```
blue2mesh/
├── src/
│   ├── lib.rs              # Main library interface
│   ├── error.rs            # Comprehensive error handling
│   ├── import.rs           # Scheme JSON parsing and validation
│   ├── extrusion.rs        # 2D to 3D extrusion algorithms
│   ├── mesh.rs             # 3D mesh representation and operations
│   ├── channels.rs         # Channel-specific mesh generation
│   ├── junctions.rs        # Junction optimization algorithms
│   ├── quality.rs          # Mesh quality validation and improvement
│   └── export.rs           # CFD and manufacturing export
├── examples/               # Comprehensive examples
├── tests/                  # Integration tests
└── benches/               # Performance benchmarks
```

### Design Patterns

- **Strategy Pattern**: Multiple extrusion algorithms for different channel types
- **Factory Pattern**: Mesh generator creation and configuration
- **Builder Pattern**: Complex mesh configuration and assembly
- **Template Method**: Standardized mesh generation workflow

## Channel Type Support

### Supported Channel Types

| Type | Description | Mesh Strategy | CFD Optimization |
|------|-------------|---------------|------------------|
| **Straight** | Simple rectangular channels | Linear extrusion | Standard quality |
| **SmoothStraight** | Rounded corners for better flow | Rounded extrusion | Enhanced quality |
| **Serpentine** | Meandering channels with turns | Adaptive sections | Turn optimization |
| **Arc** | Curved channels | Curved path following | Curvature handling |
| **Frustum** | Tapered channels | Variable cross-section | Gradual transitions |

### Junction Handling

- **Bifurcations**: Y-junctions with smooth flow splitting
- **Trifurcations**: Three-way junctions with optimized geometry
- **Complex Junctions**: Multi-channel intersections with flow optimization

## Quality Control

### Mesh Quality Metrics

- **Aspect Ratio**: Element shape quality (target: < 5.0 for CFD)
- **Angles**: Minimum/maximum angles (target: 30°-120° for CFD)
- **Skewness**: Element distortion (target: < 0.6 for CFD)
- **Manifold**: Geometric validity for manufacturing
- **Volume**: Element volume consistency

### Quality Levels

| Level | Target Application | Aspect Ratio | Min Angle | Quality Score |
|-------|-------------------|--------------|-----------|---------------|
| **Fast** | Quick prototyping | < 20.0 | > 10° | > 0.5 |
| **Balanced** | General use | < 10.0 | > 20° | > 0.7 |
| **HighQuality** | CFD simulation | < 5.0 | > 30° | > 0.8 |
| **Research** | High-accuracy CFD | < 3.0 | > 45° | > 0.9 |

## Export Formats

### CFD Formats
- **VTK**: ParaView visualization and general CFD solvers
- **OpenFOAM**: Native OpenFOAM polyMesh format
- **CGNS**: Advanced CFD applications

### Manufacturing Formats
- **STL**: 3D printing (ASCII and binary)
- **OBJ**: General 3D modeling
- **PLY**: Point cloud and mesh analysis

## Performance

### Benchmarks (Typical Performance)

| Operation | Simple Design | Complex Design | Performance Target |
|-----------|---------------|----------------|-------------------|
| **Import** | < 1 ms | < 10 ms | Sub-millisecond |
| **Extrusion** | < 10 ms | < 100 ms | Real-time |
| **Quality Check** | < 5 ms | < 50 ms | Interactive |
| **Export** | < 20 ms | < 200 ms | Near real-time |

### Memory Usage
- **Efficient**: Zero-copy operations where possible
- **Scalable**: Streaming for large meshes
- **Optimized**: SIMD and parallel processing

## Integration

### With OxiCFD
```rust
// Generate mesh optimized for CFD
let mesh = MeshGenerator::new()
    .with_quality_level(QualityLevel::HighQuality)
    .with_boundary_layers(true)
    .generate_from_design(&design)?;

// Export for oxicfd
CfdExporter::new().export_vtk(&mesh, "cfd_mesh.vtk")?;
```

### With Manufacturing
```rust
// Generate manufacturing-ready mesh
let mesh = MeshGenerator::new()
    .with_quality_level(QualityLevel::Research)
    .generate_from_design(&design)?;

// Export for 3D printing
ManufacturingExporter::new()
    .with_units(ExportUnits::Millimeters)
    .export_stl(&mesh, "device.stl")?;
```

## Examples

Run the examples to see Blue2Mesh in action:

```bash
# Basic 3D extrusion
cargo run --example basic_extrusion

# Bifurcation mesh generation
cargo run --example bifurcation_mesh

# Complete workflow with CFD
cargo run --example complete_workflow

# Manufacturing export
cargo run --example manufacturing_export
```

## Dependencies

### Core Dependencies

- **[csgrs](https://github.com/timschmidt/csgrs/tree/main/src)**: 3D geometry and boolean operations
- **[scheme](https://github.com/ryancinsight/scheme)**: 2D millifluidic design import
- **nalgebra**: Linear algebra and 3D mathematics
- **serde**: JSON serialization/deserialization

### Optional Dependencies

- **oxicfd**: CFD integration (feature: `oxicfd-integration`)
- **meshopt**: Mesh optimization (feature: `quality-control`)
- **plotters**: Visualization (feature: `visualization`)

## Contributing

Blue2Mesh follows the same development standards as the broader millifluidic design framework:

- **SOLID Principles**: Clean, maintainable architecture
- **Comprehensive Testing**: Unit, integration, and performance tests
- **Documentation**: Extensive docs with examples
- **Performance**: Optimized for real-time applications

## License

MIT License - see [LICENSE](../LICENSE) for details.

## Related Projects

- [**scheme**](https://github.com/ryancinsight/scheme): 2D millifluidic design and pattern generation
- [**csgrs**](https://github.com/timschmidt/csgrs): 3D constructive solid geometry operations
- **oxicfd**: Computational fluid dynamics framework (experimental)
