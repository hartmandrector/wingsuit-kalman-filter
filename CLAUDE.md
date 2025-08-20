# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

FlySight Plotter is a GPS trajectory visualization application built with Vite and TypeScript. It processes FlySight CSV files containing GPS data and applies motion estimation for trajectory smoothing and 60Hz interpolation between GPS measurements.

## Development Commands

- `npm run dev` - Start development server with hot reload (you should never run this, the user will)
- `npm run build` - Build production bundle
- `npm run preview` - Preview production build locally  
- `npm run test` - Run all tests with Vitest
- `npm run test:ui` - Run tests with Vitest UI interface
- `npm run typecheck` - Run TypeScript type checking

## Architecture

### Core Components

- **src/main.ts** - Main application entry point handling file upload, drag/drop UI, and processing pipeline orchestration
- **src/plotter.ts** - GPS data processing pipeline with Kalman filtering, 60Hz interpolation, and canvas rendering with zoom/pan
- **src/motionestimator.ts** - Motion estimation implementation using complementary filtering with alpha blending
- **src/enu.ts** - ENU coordinate system conversions for GPS data processing
- **src/csvParser.ts** - FlySight-specific CSV parser for $GNSS data format with robust field detection
- **src/coordinates.ts** - Coordinate system conversion between lat/lon/alt and East-North-Up (ENU) local coordinates
- **src/matrix.ts** - Matrix operations library supporting Kalman filter mathematics
- **index.html** - Drop zone UI and canvas element for visualization

### Key Classes and Functions

- `MotionEstimator` - Motion estimation with GPS input and ENU prediction in src/motionestimator.ts:38
- `parseCSV()` - FlySight format parser extracting $GNSS lines in src/csvParser.ts:1
- `plotData()` - GPS processing pipeline with Kalman filtering and 60Hz interpolation in src/plotter.ts:35
- `latLonAltToENU()` / `ENUToLatLonAlt()` - Coordinate system conversion in src/coordinates.ts:27,46
- `matrixMultiply()`, `matrixInverse3x3()` - Matrix operations in src/matrix.ts:18,74

### Data Processing Pipeline

1. User drops FlySight CSV file containing $GNSS data lines or uses default TRACK.CSV
2. CSV parser extracts GPS coordinates, timestamps, and altitude from $GNSS lines using flexible field detection
3. GPS points are converted to MLocation format for motion estimation processing
4. MotionEstimator processes GPS measurements using complementary filtering with alpha blending
5. 60Hz interpolation generates predicted positions between GPS measurements using motion state prediction
6. Both filtered GPS points (blue, 4px radius) and interpolated points (red, 1px radius) are rendered on canvas
7. Interactive zoom/pan controls allow detailed trajectory inspection

### FlySight CSV Format

The parser handles FlySight data format with fallback field detection:
- Skips first 7 header lines (metadata, column definitions, units)
- Extracts only lines beginning with `$GNSS,`
- Uses hardcoded header mapping: `['GNSS', '', 'lat', 'lon', 'hMSL', 'velN', 'velE', 'velD', 'hAcc', 'vAcc', 'sAcc', 'numSV']`
- Flexible field detection supports variations in lat/lon/alt/time column naming

### MotionEstimator Implementation

- **State Vector**: 3D position, velocity, and acceleration in ENU coordinates
- **Filtering**: Alpha blending complementary filter with configurable alpha parameter (default 0.1)
- **Input**: GPS position and velocity measurements in lat/lon/alt format
- **Motion Model**: Constant acceleration prediction with velocity taken directly from GPS
- **Coordinate System**: East-North-Up (ENU) local tangent plane relative to first GPS fix
- **Data Type**: Uses MLocation interface for consistent GPS data handling

### Testing

- **Vitest** framework with comprehensive test coverage
- **src/kalman.test.ts** - Legacy Kalman filter tests (may need updating for MotionEstimator)
- **src/csvParser.test.ts** - CSV parsing edge cases and FlySight format validation  
- **src/coordinates.test.ts** - ENU coordinate system conversion accuracy
- **src/matrix.test.ts** - Matrix operations correctness and edge cases
