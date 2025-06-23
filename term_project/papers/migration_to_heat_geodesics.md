
### Pipeline 2: Migrate to Heat Geodesics

This pipeline replaces Dijkstra-based distance calculations with the faster and more robust Heat Method within the main segmentation algorithm. This will affect both how the initial points are sampled and how the skeletal segments are scored.

**Step 1: Ensure Heat Geodesics are Precomputed**

Your `loadMesh` function in `main_segmentation.cpp` already does this, which is perfect. The `heat_data` is precomputed and stored in the `Mesh` object.

**Step 2: Integrate Heat Geodesics into Farthest Point Sampling**

Create a new FPS function that uses heat distances. You can add a menu option to let the user choose which FPS method to use during the full segmentation.

````cpp
// ...existing code...
// In the `performFullSegmentation` function:

// Ask user which distance metric to use
cout << "Choose distance metric for FPS and Descriptors:" << endl;
cout << "1. Euclidean (Fast, less accurate on complex shapes)" << endl;
cout << "2. Heat Geodesics (Slower startup, more robust)" << endl;
int choice = 2; // Or get user input
// ...

// --- STAGE 1: FARTHEST POINT SAMPLING ---
cout << "\n--- STAGE 1: FARTHEST POINT SAMPLING ---" << endl;
if (choice == 2 && g_mesh->heat_geodesics_precomputed) {
    cout << "Using Heat Geodesics for Farthest Point Sampling." << endl;
    g_segmentation_result.fpsPoints = g_mesh->farthestPointSamplingHeat(num_fps_samples);
} else {
    cout << "Using Euclidean distance for Farthest Point Sampling." << endl;
    g_segmentation_result.fpsPoints = g_mesh->farthestPointSampling(num_fps_samples);
}
// ...
````

You will need to implement `farthestPointSamplingHeat` in your Mesh.cpp file.

````cpp
// ...existing code...
std::vector<int> Mesh::farthestPointSampling(int n) {
    // ... your existing Euclidean FPS implementation
}

std::vector<int> Mesh::farthestPointSamplingHeat(int n) {
    if (!heat_geodesics_precomputed) {
        cout << "Error: Heat geodesics must be precomputed for this FPS method." << endl;
        return {};
    }
    cout << "Running Farthest Point Sampling with Heat Geodesics..." << endl;
    std::vector<int> samples;
    if (V.rows() == 0 || n <= 0) return samples;

    samples.reserve(n);

    Eigen::VectorXd min_dists = Eigen::VectorXd::Constant(V.rows(), std::numeric_limits<double>::max());

    // 1. Pick first point randomly
    int current_idx = 0;
    samples.push_back(current_idx);

    for (int i = 1; i < n; ++i) {
        // 2. Compute distances from the last added sample
        Eigen::VectorXd new_dists = computeHeatGeodesicDistances(current_idx);

        // 3. Update the minimum distance for each vertex
        for(int j = 0; j < V.rows(); ++j) {
            if(new_dists(j) < min_dists(j)) {
                min_dists(j) = new_dists(j);
            }
        }

        // 4. Find the vertex that is farthest from the current set of samples
        min_dists.maxCoeff(&current_idx);
        samples.push_back(current_idx);
    }
    return samples;
}
// ...existing code...
````

**Step 3: Integrate Heat Geodesics into the D-Descriptor**

Modify the main segmentation loop to use heat geodesics when calculating the D-Descriptor, which is used for scoring skeletal segments.

````cpp
// ...existing code...
// Inside `performFullSegmentation`, in the loop that computes pairwise harmonic fields:

// Before the loop:
Eigen::VectorXd dist_p, dist_q;
if (choice == 2 && g_mesh->heat_geodesics_precomputed) {
    cout << "Using Heat Geodesics for D-Descriptors." << endl;
}

// Inside the loop over point pairs (p, q):
// ...
Eigen::VectorXd harmonic_field = g_harmonics->computeField(p, q);
// ...
if (choice == 2 && g_mesh->heat_geodesics_precomputed) {
    dist_p = g_mesh->computeHeatGeodesicDistances(p);
    dist_q = g_mesh->computeHeatGeodesicDistances(q);
    segment.D = g_harmonics->computeDDescriptor(p, q, harmonic_field, K, &dist_p, &dist_q);
} else {
    // Fallback to Dijkstra or null
    segment.D = g_harmonics->computeDDescriptor(p, q, harmonic_field, K, nullptr, nullptr);
}
// ...
````
