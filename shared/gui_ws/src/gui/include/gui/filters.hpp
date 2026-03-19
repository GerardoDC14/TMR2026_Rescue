#pragma once

/// Register all application filters (QR Code, YOLO Detect, Detect Shape).
/// Call once before creating the MainWindow.
void registerFilters();

/// Release the YOLO CUDA session before the CUDA runtime unloads.
/// Must be called before main() returns to avoid cudaFreeHost on a
/// shut-down driver (static destructor ordering issue).
void shutdownFilters();
