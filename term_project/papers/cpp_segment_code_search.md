

# **An Implementation Blueprint: Re-enacting "Pairwise Harmonics for Simultaneous Skeletonization and Segmentation" in C++**

## **Introduction**

The 2013 paper "Pairwise Harmonics for Shape Analysis" by Zheng et al. introduced a novel and effective framework for geometry processing.1 Departing from traditional techniques that compute descriptors for individual surface points, this work pioneers a pairwise analysis approach. By defining descriptors based on the relationship between pairs of points on a surface, the framework unlocks a new class of global, discriminative features capable of capturing complex geometric variations and structures like symmetry.1 The core of the method lies in computing harmonic functions where the global extrema are constrained to a specific point pair, and then analyzing the properties of the resulting isocurves.1  
This report addresses the objective of re-enacting the "Simultaneous Skeletonization and Segmentation" algorithm, as detailed in Section 3.3 of the paper, within a C++ environment.1 A thorough investigation reveals that while the paper's authors have made source code for other research projects publicly available, a reference implementation for this specific algorithm does not appear to be in the public domain.5 Consequently, this document is designed to bridge that implementation gap. It serves as a definitive, expert-level blueprint for a computer science graduate student or software engineer to translate the academic concepts of the paper into a functional and robust C++ program.  
The following sections will provide a comprehensive, step-by-step roadmap. The journey begins with the selection and setup of a modern C++ geometry processing toolkit, proceeds through the implementation of the core pairwise harmonic field engine and its associated descriptors, and culminates in the assembly of the complete segmentation and skeletonization pipeline. Each step is grounded in the theory presented in the paper and augmented with practical implementation details, code logic, and discussions of the underlying mathematical principles.

## **Section 1: The Modern Geometry Processing Toolkit in C++**

To successfully translate the algorithms from "Pairwise Harmonics for Shape Analysis" into code, a foundational software stack is required. The choice of libraries is a strategic decision that directly impacts development speed, code clarity, and performance. This section outlines a recommended toolkit, justifying each component based on the specific demands of the paper's methods.

### **1.1 The Core Stack: Selecting libigl and Eigen**

For this project, the combination of the **libigl** geometry processing library and the **Eigen** linear algebra library represents the most direct and effective path to implementation.20  
**libigl** is a C++ library designed for geometry processing research. Its key advantages for this task include:

* **Header-Only Architecture:** libigl does not require a complex pre-compilation or installation process. Including its headers in a project is sufficient to gain access to its functionality, which significantly lowers the barrier to entry.20  
* **Matrix-Based API:** The library's functions operate on simple matrix and vector data structures provided by Eigen. This design philosophy aligns perfectly with the mathematical notation common in academic papers, allowing for a near one-to-one translation from formula to code.21  
* **Essential Operators:** It provides direct, efficient implementations of the core discrete differential geometry operators required by the paper, most notably the cotangent Laplacian (igl::cotmatrix) and a high-level solver for harmonic functions (igl::harmonic).20

**Eigen** is a high-performance C++ template library for linear algebra that serves as the foundation for libigl.20 Its relevance is paramount:

* **Sparse Matrix Module:** The paper's central computation involves solving a large, sparse linear system of the form Lf=b.1 Eigen's  
  SparseMatrix class and its associated methods are optimized for the storage and manipulation of such matrices.22  
* **Direct and Iterative Solvers:** Eigen provides a suite of robust numerical solvers. For this application, its sparse direct solvers, such as Eigen::SimplicialLLT (for symmetric positive definite systems) and Eigen::SparseLU, are critical for efficiently computing the harmonic fields.22

The symbiotic relationship between these two libraries is a powerful accelerator for research implementation. The paper's mathematical constructs, such as the Laplacian matrix L, are directly produced as Eigen matrices by libigl functions, minimizing the cognitive overhead and potential for error that can arise from more abstract library designs.

### **1.2 Representing 3D Meshes: The Vertex-Face Matrix Paradigm**

libigl represents a triangle mesh using a simple and powerful paradigm: a pair of Eigen matrices, commonly denoted (V, F).21

* Eigen::MatrixXd V: A matrix of doubles with \#V rows and 3 columns, where \#V is the number of vertices. Each row V.row(i) stores the (x, y, z) coordinates of the i-th vertex.  
* Eigen::MatrixXi F: A matrix of integers with \#F rows and 3 columns, where \#F is the number of triangular faces. Each row F.row(j) contains the three integer indices of the vertices that form the j-th triangle. These indices correspond to the rows of the V matrix.

This pointer-free representation offers significant advantages over more complex data structures like the half-edge model. It is memory-efficient, benefits from contiguous memory layouts for better cache performance, and can be trivially copied, saved to a file, or sent over a network, which greatly simplifies debugging and data management.21

### **1.3 Essential Mathematical Machinery: Sparse Solvers and Geometric Operators**

The implementation will rely on a small but powerful set of functions from the chosen toolkit. A preliminary map of these functions includes:

* igl::cotmatrix(V, F, L): Computes the sparse cotangent Laplacian matrix L from the mesh geometry (V, F).20  
* igl::massmatrix(V, F, type, M): Computes the sparse mass matrix M, which assigns a weight (e.g., Voronoi area) to each vertex. This is needed for certain numerical formulations.25  
* igl::harmonic(...): A high-level function that encapsulates the entire process of solving the Laplace equation with specified Dirichlet boundary conditions, providing a convenient "black box" solution.26  
* Eigen::SparseMatrix\<double\>: The fundamental data structure used to store L and M.22  
* Eigen::SimplicialLLT\<Eigen::SparseMatrix\<double\>\>: A direct sparse Cholesky factorization solver, ideal for the symmetric positive definite systems that arise after applying boundary conditions.22  
* Eigen::SparseLU\<Eigen::SparseMatrix\<double\>\>: A more general direct sparse LU factorization solver that can also be used.22

### **1.4 Augmenting the Toolkit: Specialized Libraries**

While libigl and Eigen form the core stack, awareness of other powerful libraries is valuable for context and for potentially sourcing specific, highly-optimized components.

* **CGAL (Computational Geometry Algorithms Library):** CGAL is an immensely powerful and comprehensive library known for its robustness and geometric correctness. It offers a vast suite of algorithms for mesh processing, repair, Boolean operations, and geometric queries.33 However, its use of more complex data structures (like  
  Surface\_mesh which is based on a half-edge representation) and a steeper learning curve make it less direct for this specific re-enactment task compared to libigl's matrix-centric approach.34  
* **Geometry-Central:** This is a modern, high-performance C++ library developed with a focus on cutting-edge research algorithms. It features excellent, state-of-the-art implementations of methods like the Heat Method for geodesic distance and parallel transport, which are directly relevant to calculating one of the paper's key descriptors.38  
* **Geodesic Distance Libraries:** The computation of geodesic distance (shortest path on the mesh surface) is a classic problem. Specialized libraries exist that provide highly optimized implementations of algorithms like the Mitchell, Mount, and Papadimitriou (MMP) algorithm for exact distances or faster approximate methods.42

The following table summarizes the comparison and justifies the selection of libigl as the primary tool.  
**Table 1: Comparison of C++ Geometry Processing Libraries for This Project**

| Library Name | Primary Strengths | Key Data Structures | Dependencies | Suitability for this Project |
| :---- | :---- | :---- | :---- | :---- |
| **libigl** | Rapid prototyping, direct mapping of math to code, minimal setup. | Eigen::MatrixXd (Vertices), Eigen::MatrixXi (Faces) | **Core:** Eigen (header-only). **Optional:** Other libraries for extended features. | **Optimal.** The API directly mirrors the paper's linear algebra formulation (Lf=b), making it the fastest and most straightforward choice for re-enacting the research. |
| **CGAL** | Geometric robustness, comprehensive algorithm suite, certified correctness. | Surface\_mesh (Half-edge based), Polyhedron\_3 | Boost, GMP, MPFR. Requires linking. | **Viable but more complex.** Would require more boilerplate code to assemble the sparse matrices from its iterators, creating an abstraction layer between the paper's math and the code. Best for tasks requiring extreme geometric robustness or complex combinatorial operations. |
| **Geometry-Central** | State-of-the-art algorithms, high performance, modern C++ design. | Custom mesh and geometry classes. | Eigen, and others. | **Excellent as a supplementary tool.** Its heat method implementation is a prime candidate for computing the geodesic distance descriptor efficiently. Less suited as the primary framework due to a different API philosophy than libigl's direct matrix approach. |

## **Section 2: From Theory to Code: Implementing Pairwise Harmonic Fields**

This section provides a detailed breakdown of the implementation of the core engine described in Section 2 of the paper: the pairwise harmonic field.1 This field is the fundamental building block from which all subsequent descriptors and algorithms are derived.

### **2.1 The Heart of the Matter: The Discrete Cotangent Laplacian**

The paper defines a harmonic function as one that satisfies the Laplace equation Δf=0.1 To apply this concept to a discrete triangle mesh, the continuous Laplace-Beltrami operator must be discretized. The standard and most widely accepted method in geometry processing, used in the paper, is the  
**cotangent-weighted Laplacian**.1  
The elegance of the cotangent formulation is that the weight for an edge connecting vertices i and j depends only on the two angles opposite that edge. Specifically, the off-diagonal entry of the Laplacian matrix L corresponding to the edge (i, j) is given by:  
Lij​=21​(cotαij​+cotβij​)  
where αij​ and βij​ are the two angles opposite the edge (i, j) in the two triangles that share it. The diagonal entries Lii​ are then defined as the negative sum of all other entries in the same row, Lii​=−∑j=i​Lij​, ensuring that the rows sum to zero.46 Note that some literature omits the  
1/2 factor; this is a valid scaling, but it's important to be consistent. The libigl implementation igl::cotmatrix computes this "half-cotangent" version, which is compatible with a Voronoi area mass matrix.47  
This formulation is favored for its desirable properties, including inherent smoothness of the resulting fields, invariance to isometric transformations, and robustness to mesh tessellation changes, as demonstrated in Figure 2 of the paper.1  
In C++, computing this matrix with libigl is a single line of code:

C++

// V: \#V x 3 matrix of vertex positions  
// F: \#F x 3 matrix of face indices  
Eigen::SparseMatrix\<double\> L;  
igl::cotmatrix(V, F, L);

This function populates the sparse matrix L with the cotangent weights, forming the discrete Laplace-Beltrami operator for the input mesh.

### **2.2 Setting Constraints: A Deep Dive into Dirichlet Boundary Conditions**

The "pairwise harmonic" fp,q​ is defined by specific **Dirichlet boundary conditions**: the scalar field value is fixed to 0 at a source vertex p and to 1 at a sink vertex q.1 All other vertices have unknown values that must be solved for.  
To enforce these constraints on the linear system Lf=b, a standard algebraic manipulation is performed. The vertices of the mesh are partitioned into two sets: the boundary vertices b (in this case, the indices of p and q) and the interior, or unknown, vertices u. The system can be rewritten in block form:  
$$ \\begin{bmatrix} L\_{uu} & L\_{ub} \\ L\_{bu} & L\_{bb} \\end{bmatrix} \\begin{bmatrix} f\_u \\ f\_b \\end{bmatrix} \= \\begin{bmatrix} 0 \\ b\_b \\end{bmatrix} $$  
Here, fu​ is the vector of unknown values we want to find, and fb​ is the vector of known boundary values (e.g., T). Since we are only solving for the interior vertices, we can expand the top row of the block matrix equation:  
Luu​fu​+Lub​fb​=0  
Rearranging to solve for the unknowns fu​ gives the final, reduced linear system:  
Luu​fu​=−Lub​fb​  
This is a smaller, non-singular, and typically symmetric positive-definite system that can be solved efficiently. The right-hand side, −Lub​fb​, effectively encodes the influence of the fixed boundary values on their interior neighbors. Once fu​ is found, it is combined with the known fb​ to form the complete scalar field f.

### **2.3 Solving the System: Assembling and Solving Lf=b with Eigen**

The process of solving for the harmonic field can be encapsulated in a C++ function. The core steps involve constructing the Laplacian, applying the boundary conditions, and invoking a sparse solver.  
For efficient sparse matrix construction, Eigen's Triplet list is the preferred method. A std::vector of Eigen::Triplet\<double\> objects is created, where each triplet stores a (row, column, value) entry for the matrix. This list is then used to initialize the SparseMatrix in one pass.22  
The following C++ logic outlines a function to compute a pairwise harmonic field:

C++

\#include \<igl/cotmatrix.h\>  
\#include \<igl/slice.h\>  
\#include \<igl/slice\_into.h\>  
\#include \<Eigen/Sparse\>  
\#include \<vector\>

// Computes the pairwise harmonic field f\_pq on a mesh (V, F) with  
// f(p\_idx) \= 0 and f(q\_idx) \= 1\.  
Eigen::VectorXd compute\_harmonic\_field(  
    const Eigen::MatrixXd& V,  
    const Eigen::MatrixXi& F,  
    int p\_idx,  
    int q\_idx)  
{  
    // 1\. Compute the full cotangent Laplacian  
    Eigen::SparseMatrix\<double\> L;  
    igl::cotmatrix(V, F, L);

    // 2\. Identify boundary and interior vertices  
    Eigen::VectorXi b(2); // Boundary indices  
    b \<\< p\_idx, q\_idx;  
      
    std::vector\<int\> all\_indices(V.rows());  
    std::iota(all\_indices.begin(), all\_indices.end(), 0);  
    std::vector\<int\> interior\_indices;  
    std::sort(b.data(), b.data() \+ b.size()); // Sort boundary indices for efficient lookup  
    std::set\_difference(all\_indices.begin(), all\_indices.end(),  
                        b.data(), b.data() \+ b.size(),  
                        std::back\_inserter(interior\_indices));  
    Eigen::VectorXi u \= Eigen::Map\<Eigen::VectorXi\>(interior\_indices.data(), interior\_indices.size());

    // 3\. Construct the reduced system L\_uu \* f\_u \= \-L\_ub \* f\_b  
    Eigen::SparseMatrix\<double\> L\_uu, L\_ub;  
    igl::slice(L, u, u, L\_uu);  
    igl::slice(L, u, b, L\_ub);

    Eigen::VectorXd f\_b(2);  
    f\_b \<\< 0.0, 1.0; // Boundary values f(p)=0, f(q)=1

    Eigen::VectorXd rhs \= \-L\_ub \* f\_b;

    // 4\. Solve the linear system for interior values  
    Eigen::SimplicialLLT\<Eigen::SparseMatrix\<double\>\> solver;  
    solver.compute(L\_uu);  
    if(solver.info()\!= Eigen::Success) {  
        // handle error, matrix is not positive definite  
        throw std::runtime\_error("Cholesky decomposition failed.");  
    }  
    Eigen::VectorXd f\_u \= solver.solve(rhs);  
    if(solver.info()\!= Eigen::Success) {  
        // handle error in solve stage  
        throw std::runtime\_error("Solver failed.");  
    }

    // 5\. Combine boundary and interior values into the final field  
    Eigen::VectorXd f(V.rows());  
    igl::slice\_into(f\_u, u, 1, f);  
    igl::slice\_into(f\_b, b, 1, f);

    return f;  
}

### **2.4 The igl::harmonic Function: A High-Level Abstraction**

After understanding the underlying mechanics, it is instructive to use the high-level abstraction provided by libigl. The igl::harmonic function automates the entire process of setting up and solving the constrained system.26  
Its usage is as follows:

C++

// V, F, p\_idx, q\_idx are as before  
Eigen::VectorXi b(2);  
b \<\< p\_idx, q\_idx;

Eigen::MatrixXd bc(2, 1);  
bc \<\< 0.0, 1.0; // Boundary values

Eigen::MatrixXd W; // Output weights/field

// Solve using cotangent weights (k=1 for harmonic)  
igl::harmonic(V, F, b, bc, 1, W);

Here, b contains the indices of the boundary vertices, bc contains the corresponding fixed values, k=1 specifies a harmonic (as opposed to biharmonic, etc.) problem, and the resulting scalar field is stored in the output matrix W.26 This function provides a powerful and convenient way to compute the fields once the core principles are understood.  
The power of the pairwise harmonic method comes from this very specific problem formulation. A general harmonic field can have complex behavior. However, by constraining the field to have its global minimum and maximum at exactly two points, p and q, the method transforms the Laplacian from a generic smoothing operator into a highly specific probe.1 The resulting isocurves are not arbitrary level sets; they represent a "wavefront" propagating across the surface from  
p to q. The underlying geometry of the shape—its bulges, constrictions, and holes—distorts these wavefronts. This distortion is precisely what the descriptors in the next section are designed to quantify.

## **Section 3: Extracting Meaning: Isocurves and Geodesic Descriptors**

Once the pairwise harmonic scalar field fp,q​ has been computed for a mesh, the next stage is to extract the geometric features that form the basis of the paper's descriptors. This involves generating isocurves from the scalar field and measuring their properties.

### **3.1 Isocontouring on Triangle Meshes: An Implementation of Marching Squares**

An isocurve, or level set, for a value isoval is the collection of points on the surface where the scalar field equals that value.1 On a discrete triangle mesh where the scalar field is defined at the vertices, an isocurve becomes a set of polylines that cut across the faces of the mesh. The standard algorithm to extract these polylines is a 2D analog of Marching Cubes, often called  
**Marching Squares on Triangles**.51  
The algorithm proceeds on a per-triangle basis:

1. For each triangle in the mesh, retrieve the scalar field values at its three vertices, (f0​,f1​,f2​).  
2. Compare the target isoval with the values at the vertices. If isoval is not between the minimum and maximum of (f0​,f1​,f2​), the isocurve does not pass through this triangle.  
3. If the isocurve does intersect the triangle, it must cross exactly two of its edges.  
4. For each of the two intersected edges (e.g., the edge between vertex i and vertex j), the intersection point is found by linear interpolation. If V\_i and V\_j are the vertex positions, the intersection point P\_ij is:  
   Pij​=Vi​+fj​−fi​isoval−fi​​(Vj​−Vi​)  
5. A single line segment is created by connecting the two intersection points found within the triangle.  
6. After processing all triangles, the result is a collection of disconnected line segments. These segments must be stitched together into one or more continuous polylines. This is typically done by building a connectivity graph of the segment endpoints and traversing it.

While libraries like CGAL provide sophisticated tools for isocontouring 52, a direct implementation is feasible. The  
MeshUtility library also provides an IsoCurve function that encapsulates this logic.56 A C++ function  
extract\_isocurve(V, F, f, isoval) would take the mesh, the scalar field, and an isovalue, and return a std::vector of polylines, where each polyline is a std::vector of 3D points.

### **3.2 Calculating the Perimeter Descriptor (Rpq​)**

The **Perimeter Descriptor**, Rpq​, captures the cross-sectional profile of the shape along the diffusion path from p to q. It is defined as the distribution of the lengths of K isocurves uniformly sampled from the harmonic field.1  
The implementation follows directly from the isocontouring step:

1. Define the number of isocurves to sample, K (the paper uses K=16 for symmetry and matching applications, and K=50 for skeletonization).1  
2. Iterate K times, with the isovalue isoval stepping uniformly from a value just above 0 to just below 1 (e.g., from 1/(K+1) to K/(K+1)).  
3. In each iteration, call the extract\_isocurve function to get the polylines for the current isoval.  
4. The length of the i-th isocurve, rpqi​, is the sum of the lengths of all polylines generated for that isovalue. The length of a single polyline is the sum of the Euclidean distances between its consecutive points.  
5. The resulting vector Rpq​={rpq1​,rpq2​,...,rpqK​} is the perimeter descriptor.

### **3.3 The Geodesic Distance Challenge and the Distance Descriptor (Dpq​)**

The **Distance Descriptor**, Dpq​, is designed to capture information orthogonal to the perimeter descriptor. It measures the "stretch" of the diffusion path by computing the average **geodesic distance** from points on an isocurve back to the source or sink vertex.1  
Geodesic distance is the shortest-path distance between two points constrained to lie on the surface of the mesh. This is a fundamentally different and more complex computation than simple Euclidean distance in 3D space.  
Implementation Options and Trade-offs:  
The choice of algorithm for computing geodesic distance involves a critical trade-off between accuracy and performance.

* **Option A (Exact):** The classic algorithm by Mitchell, Mount, and Papadimitriou (MMP) computes the exact polyhedral geodesic distance. Implementations exist, such as the one originally hosted on code.google.com/p/geodesic and now available in various forks and wrappers like tvb-gdist.42 The primary drawback is its computational complexity, which can be slow for the large number of queries required to compute the descriptor.  
* **Option B (Approximate \- Recommended):** The **Heat Method** is a popular and much faster alternative that provides high-quality approximations of geodesic distance.41 It works by simulating heat flow for a short amount of time and then solving a Poisson equation. This method is well-suited for descriptor computation where extreme precision is less critical than speed. Both  
  **libigl** (igl::heat\_geodesic) and **Geometry-Central** (HeatMethodDistanceSolver) provide robust implementations.41

**C++ Implementation Logic for Dpq​:**

1. **Precomputation:** Using the chosen geodesic algorithm (e.g., the Heat Method), compute two full distance fields: one originating from vertex p and another from vertex q. This gives the geodesic distance from p and q to every other vertex on the mesh.  
2. **Descriptor Calculation:** Loop K times to compute each element dpqi​ of the descriptor.  
3. For the i-th isocurve (corresponding to isoval \= i/(K+1)), extract its constituent points using the Marching Squares method.  
4. For each point on the isocurve, determine its geodesic distance to the relevant source. Since the point lies on an edge of the original mesh, its distance can be found by linearly interpolating the pre-computed geodesic distances at the edge's endpoints.  
5. If i\<K/2, use the distance field computed from source p. If i≥K/2, use the distance field from source q.  
6. Average all the computed geodesic distances for all points on the i-th isocurve. This average is dpqi​.  
7. The resulting vector Dpq​={dpq1​,dpq2​,...,dpqK​} is the distance descriptor.

These two descriptors, Rpq​ and Dpq​, provide a rich signature of the geometry between the point pair. Rpq​ measures the "width" of the shape along the path, sensitive to features like limbs or bulges. Dpq​ measures the "length" of the path, sensitive to how it meanders or curves across the surface. This orthogonality is what gives the pairwise harmonic framework its descriptive power, enabling it to effectively distinguish between different shape parts and detect complex relationships like symmetry.1

## **Section 4: The Full Pipeline: A C++ Guide to Simultaneous Segmentation and Skeletonization**

This section synthesizes the previously developed components into a complete C++ implementation of the "Simultaneous Skeletonization and Segmentation" algorithm from Section 3.3 of the paper.1 This algorithm elegantly leverages the pairwise harmonic isocurves to infer both a 1D skeleton and a 3D surface partition. The process is a prime example of a "structure-from-behavior" approach: it does not search for structure directly but infers it from the emergent properties of many simulated diffusion processes.

### **4.1 Step 1: Surface Sampling**

The algorithm begins not by analyzing the entire surface, but by selecting a sparse yet representative set of points.

* **Goal:** Obtain a set of points, denoted Δ, that are well-distributed and located on prominent parts of the shape.1  
* **Method:** The paper specifies a "max-min sampling strategy".1 This is a greedy algorithm that iteratively selects the point that is farthest from the set of already-selected points.  
* **Implementation:** The libigl library provides a direct implementation of this strategy: igl::max\_min\_dist. This function can be used to select a desired number of sample points from the input mesh vertices V.

C++

\#include \<igl/max\_min\_dist.h\>

// V: \#V x 3 vertex positions  
// num\_samples: The desired number of sparse points  
Eigen::VectorXi sample\_indices;  
igl::max\_min\_dist(V, num\_samples, sample\_indices);  
// sample\_indices now holds the vertex indices of the set Delta

### **4.2 Step 2 & 3: Generating Skeletal Segments and Quantifying Rigidity**

From the sampled points, a large set of potential skeletal segments is generated. Each segment's "rigidity" is then quantified to distinguish between limb-like parts and joint-like parts.

* **Skeletal Segments:** For every pair of points (p,q) in the sampled set Δ, a pairwise harmonic field is computed. The isocurves are extracted (the paper uses K=50 for this application 1), and the 3D centroid of each isocurve is calculated. The ordered sequence of these  
  K centroids forms a single potential skeletal segment.1  
* Node Rigidity: The algorithm then assesses the local geometry of the collection of all skeletal nodes (all centroids from all segments). For each node j, its rigidity ξj​ is computed via Principal Component Analysis (PCA) on its neighboring nodes. The formula is:  
  ξj​=λ1​+λ2​+λ3​max{λ1​,λ2​,λ3​}​

  where λk​ are the eigenvalues of the covariance matrix of the neighboring nodes' positions.1 A high value (close to 1\) indicates that the neighbors are arranged linearly, suggesting a rigid, limb-like region. A lower value indicates a more planar or volumetric arrangement, characteristic of a joint.  
* **C++ Logic:**  
  1. Create a master list to store all skeletal nodes (e.g., std::vector\<Eigen::Vector3d\> all\_nodes).  
  2. Loop through all pairs of indices in sample\_indices. For each pair:  
     a. Compute the harmonic field and extract K isocurves.  
     b. For each isocurve, calculate its centroid and add it to all\_nodes. Store the sequence of centroids as one skeletal segment.  
  3. Build a spatial data structure (e.g., a k-d tree) on all\_nodes for efficient neighborhood queries.  
  4. Loop through each node j in all\_nodes.  
     a. Find all neighboring nodes within a specified radius (the paper suggests using the radius of the associated isocurve 1).

     b. Form a matrix of these neighbors' 3D positions.  
     c. Compute the covariance matrix and use Eigen's SVD (Eigen::JacobiSVD) to find its principal components (related to the eigenvalues λk​).  
     d. Calculate and store the rigidity score ξj​.  
* **Segment Division:** After computing rigidity for all nodes, iterate through the stored skeletal segments. Each segment is split into subsegments at any node where the rigidity ξj​ is below a threshold (the paper uses 0.9).1

### **4.3 Step 4, 5, & 6: Segment Scoring and Greedy Construction of the Initial Skeleton**

The collection of subsegments is filtered to find the most prominent ones that will form the initial skeleton.

* **Scoring:** Each subsegment s is assigned a score ρs​ that balances its rigidity and length 1:ρs​=ξs​⋅ls​

  where ξs​ is the average rigidity of all nodes on the segment and ls​ is its Euclidean length.  
* **Greedy Selection:** A greedy algorithm selects the final set of skeletal segments:  
  1. Create a list of all subsegments and their scores.  
  2. Sort this list in descending order based on ρs​.  
  3. Initialize an empty set for the final skeleton.  
  4. Iterate through the sorted list. For each candidate segment, check if it "overlaps" with any segment already in the final set. The paper suggests that overlap is based on proximity, so a simple check could be to see if the average distance between the nodes of the candidate segment and the nodes of any selected segment is below a threshold.  
  5. If there is no significant overlap, add the candidate segment to the final skeleton. This process ensures that, for example, only one primary segment is chosen to represent an arm, even if multiple were generated.1

### **4.4 Step 7: Initial Mesh Partitioning via Isocurve Boundaries**

This is a conceptually simple but practically challenging step: cutting the mesh to create an initial segmentation.

* **Concept:** The paper states the mesh is partitioned "using the isocurves associated with the two end nodes of each \[selected\] skeletal segment".1 These isocurves act as cutting boundaries.  
* **Implementation Challenge:** This is a mesh surgery operation. The isocurves are polylines whose vertices lie *on the edges* of the original mesh, not at its vertices. To perform the cut, the mesh topology and geometry must be modified:  
  1. For each cutting isocurve, its vertices must be added to the mesh's vertex list V.  
  2. The original mesh edges that contain these new vertices must be split.  
  3. The face connectivity matrix F must be updated to reference the new vertices, effectively creating new triangles and seams along the cut.  
  4. Once all cuts are made, the mesh will consist of several disconnected patches. A connected components algorithm (e.g., a flood-fill on the face adjacency graph) can be used to assign a unique ID to each patch, yielding the initial segmentation.  
* **Library Support:** This is an advanced operation. While libigl provides some cutting tools, they may not be directly suited for this arbitrary polyline cutting. CGAL offers more robust (but also more complex) Boolean and mesh corefinement operations that could be adapted for this purpose.36 A manual implementation would require careful management of the mesh data structures.

### **4.5 Step 8 & 9: Completing the Skeleton and Refining the Final Segmentation**

The final steps refine both the skeleton and the segmentation based on their mutual relationship.

* **Skeleton Completion:** The initial segmentation will have patches corresponding to the selected skeletal segments, and other patches corresponding to junctions or endpoints. The centers of these junction/endpoint patches are computed and connected to the nearby endpoints of the initial skeletal segments, guided by the adjacency of the segmented patches. This completes the skeleton graph.1  
* **Segmentation Refinement:** The segmentation is then refined using the completed skeleton as a guide. The paper describes a merging process: components at the tips of the skeleton (e.g., hands) are merged with their single neighbor. Then, components that lie along skeletal branches connecting two junctions (e.g., a torso) are considered for merging with their adjacent junction components, with the process prioritized by component volume.1 This cleans up the segmentation to better reflect the articulated structure.

The following table provides a direct mapping from the algorithm's steps to recommended C++ implementations.  
**Table 2: Algorithm-to-Code Implementation Map for Segmentation/Skeletonization**

| Algorithm Step 1 | Key Formula/Concept | Recommended C++ Library/Function | Notes/Parameters |
| :---- | :---- | :---- | :---- |
| 1\. Surface Sampling | Max-min distance sampling | igl::max\_min\_dist(V, num\_samples,...) | num\_samples controls density. |
| 2\. Skeletal Segment Generation | Centroids of isocurves from pairwise harmonics | Custom loop using compute\_harmonic\_field and extract\_isocurve | K=50 isocurves per field. |
| 3\. Node Rigidity & Division | PCA on node neighbors: ξj​ | Custom loop with Eigen::JacobiSVD for PCA | Rigidity threshold 0.9. |
| 4\. Segment Scoring | ρs​=ξs​⋅ls​ | Custom loop calculating average rigidity and length | \- |
| 5\. Greedy Skeleton Construction | Sort by ρs​ and select non-overlapping segments | std::sort and a custom overlap check | Overlap definition is key. |
| 6\. Initial Segmentation | Cut mesh along isocurves at segment ends | Complex mesh surgery. Potentially CGAL or custom implementation. | This is the most complex geometric step. |
| 7\. Skeleton Completion | Connect centroids of junction patches | Graph traversal on segmented patch adjacency | \- |
| 8\. Segmentation Refinement | Merge components based on skeleton structure | Custom merging logic based on adjacency and volume | \- |

## **Section 5: Practical Considerations and Advanced Topics**

Implementing a research paper involves more than just translating algorithms; it requires addressing real-world performance, parameterization, and robustness challenges. This section provides expert guidance on these practical aspects.

### **5.1 Performance Bottlenecks and Optimization Strategies**

The most computationally intensive part of the entire pipeline is the repeated calculation of pairwise harmonic fields. For a set of N sampled points, this requires on the order of N2/2 sparse linear solves.

* **Bottleneck Identification:** The dominant cost in each compute\_harmonic\_field call is not the final substitution step, but the initial factorization of the sparse matrix L\_uu.  
* **Optimization via Pre-factorization:** The key insight is that the Laplacian matrix L depends only on the mesh geometry, which is constant. While the specific submatrix L\_uu changes for each point pair (since the boundary vertices b change), a significant speedup can still be achieved. The paper alludes to this by mentioning "fast Choleskey factorization updating schemes".1 A more straightforward approach is to use a direct solver that allows for pre-factorization. With  
  Eigen::SparseLU or Eigen::SimplicialLLT, the compute() phase performs the expensive factorization. Subsequent calls to solve() for different right-hand sides (which change with each point pair) are much faster, consisting only of forward and backward substitutions.22 For problems where the set of interior vertices  
  *is* fixed, the factorization can be computed once and reused many times, offering a dramatic performance increase.

### **5.2 A Guide to Parameter Tuning**

The algorithm's behavior is governed by several key parameters. The paper provides values used in its experiments, which serve as an excellent starting point.1

* **Number of Sampled Points (N):** This controls the density of the initial skeletal segment generation. A higher N provides a more thorough exploration of the shape but quadratically increases the number of harmonic fields to compute. A value between 100-300 is a reasonable starting point for moderately complex meshes.  
* **Number of Isocurves (K):** This determines the resolution of each skeletal segment. The paper uses K=50 for skeletonization to get a fine-grained representation of the segments.1 For other applications like symmetry, a smaller  
  K (e.g., 16\) is sufficient.  
* **Rigidity Threshold:** The paper uses a fixed threshold of 0.9 to classify nodes as non-rigid. This value works well for many models but may need adjustment for shapes with very subtle or very pronounced joints. A lower value (e.g., 0.85) will classify more nodes as rigid, potentially helping to identify less prominent branches, while a higher value (e.g., 0.95) will lead to a sparser set of non-rigid nodes, focusing only on major articulations.1

### **5.3 Error Handling and Mesh Quality Considerations**

The numerical methods used are not immune to issues arising from input data quality.

* **Mesh Quality:** The cotangent Laplacian is known to be sensitive to poor-quality meshes, particularly those with highly obtuse or degenerate "sliver" triangles, which can lead to negative cotangent weights and potential instabilities in the solver.57 It is advisable to pre-process input meshes to improve triangle quality through remeshing if they are known to be of low quality. Advanced methods exist, such as the robust Laplacian of, which is designed to handle non-manifold and low-quality meshes gracefully.41  
* **Manifold and Watertight Meshes:** The theoretical underpinnings of the Laplace-Beltrami operator assume the mesh is a 2-manifold (every edge is shared by at most two faces) and watertight (has no boundary edges). While the algorithms may still run on meshes with holes, the results can be unpredictable as the harmonic field may "leak" through the boundaries. Pre-processing steps to fill holes or repair non-manifold vertices using libraries like CGAL can significantly improve the reliability and correctness of the final output.

## **Conclusion and Future Directions**

This report has provided a comprehensive implementation blueprint for re-enacting the "Simultaneous Skeletonization and Segmentation" algorithm from "Pairwise Harmonics for Shape Analysis." The journey progresses logically from establishing a robust C++ development environment with libigl and Eigen, to the detailed implementation of the core pairwise harmonic field engine, and finally to the assembly of the full segmentation pipeline. By deconstructing the paper's theory into actionable C++ logic and addressing practical considerations like performance and parameter tuning, this document serves to bridge the gap between academic research and working code.  
The elegance of the pairwise harmonics framework lies in its simplicity and power. By reformulating shape analysis as a problem of probing the geometry *between* point pairs, it provides a discriminative and global perspective that is difficult to achieve with traditional point-based descriptors.1 The "structure-from-behavior" philosophy of the segmentation algorithm, where the skeleton and segments emerge from the collective properties of many diffusion simulations, is a particularly potent concept.  
The C++ engine developed by following this guide is not limited to the single application of segmentation. The core components—the harmonic field solver and the descriptor extractors—are the foundation for all three applications presented in the original paper. A natural future direction would be to leverage this engine to implement the algorithms for intrinsic reflectional symmetry axis computation (Section 3.1) and matching shape extremities (Section 3.2), thereby fully re-enacting the contributions of this seminal work. Further exploration could involve extending the pairwise analysis to a multi-way analysis, as suggested by the authors, potentially revealing even richer geometric and semantic information from 3D shapes.1

#### **Works cited**

1. Pairwise\_Harmonics\_for\_Shape\_Analysis.pdf  
2. Pairwise Harmonics for Shape Analysis \- College of Engineering | Oregon State University, accessed on June 21, 2025, [https://web.engr.oregonstate.edu/\~zhange/images/pairwise\_harmonics.pdf](https://web.engr.oregonstate.edu/~zhange/images/pairwise_harmonics.pdf)  
3. Pairwise Harmonics for Shape Analysis \- Oregon State University, accessed on June 21, 2025, [https://ir.library.oregonstate.edu/downloads/b2773w89g](https://ir.library.oregonstate.edu/downloads/b2773w89g)  
4. Pairwise harmonics for shape analysis \- PubMed, accessed on June 21, 2025, [https://pubmed.ncbi.nlm.nih.gov/23661011/](https://pubmed.ncbi.nlm.nih.gov/23661011/)  
5. DiffLocks: Generating 3D Hair from a Single Image using Diffusion Models \- CVF Open Access, accessed on June 21, 2025, [https://openaccess.thecvf.com/content/CVPR2025/html/Rosu\_DiffLocks\_Generating\_3D\_Hair\_from\_a\_Single\_Image\_using\_Diffusion\_CVPR\_2025\_paper.html](https://openaccess.thecvf.com/content/CVPR2025/html/Rosu_DiffLocks_Generating_3D_Hair_from_a_Single_Image_using_Diffusion_CVPR_2025_paper.html)  
6. Youyi Zheng \- CatalyzeX, accessed on June 21, 2025, [https://www.catalyzex.com/author/Youyi%20Zheng](https://www.catalyzex.com/author/Youyi%20Zheng)  
7. Youyi Zheng | Papers With Code, accessed on June 21, 2025, [https://paperswithcode.com/author/youyi-zheng](https://paperswithcode.com/author/youyi-zheng)  
8. Youyi Zheng's research works | Zhejiang University and other places \- ResearchGate, accessed on June 21, 2025, [https://www.researchgate.net/scientific-contributions/Youyi-Zheng-2143066319](https://www.researchgate.net/scientific-contributions/Youyi-Zheng-2143066319)  
9. Chiew-Lan Tai | Papers With Code, accessed on June 21, 2025, [https://paperswithcode.com/author/chiew-lan-tai](https://paperswithcode.com/author/chiew-lan-tai)  
10. Eugene Zhang \- DBLP, accessed on June 21, 2025, [https://dblp.org/pid/68/5017](https://dblp.org/pid/68/5017)  
11. Eugene Zhang Eugene6174 \- GitHub, accessed on June 21, 2025, [https://github.com/Eugene6174](https://github.com/Eugene6174)  
12. Eugene Zhang, accessed on June 21, 2025, [https://eugenezhang.me/](https://eugenezhang.me/)  
13. Eugene Zhang Ezzhingy \- GitHub, accessed on June 21, 2025, [https://github.com/Ezzhingy](https://github.com/Ezzhingy)  
14. Pengfei Xu \- ACL Anthology, accessed on June 21, 2025, [https://aclanthology.org/people/p/pengfei-xu/](https://aclanthology.org/people/p/pengfei-xu/)  
15. Pengfei Xu's research works \- ResearchGate, accessed on June 21, 2025, [https://www.researchgate.net/scientific-contributions/Pengfei-Xu-2162319508/publications/2](https://www.researchgate.net/scientific-contributions/Pengfei-Xu-2162319508/publications/2)  
16. Pengfei Xu \- CatalyzeX, accessed on June 21, 2025, [https://www.catalyzex.com/author/Pengfei%20Xu](https://www.catalyzex.com/author/Pengfei%20Xu)  
17. Pengfei Su's Homepage, accessed on June 21, 2025, [https://pengfei-su.github.io/](https://pengfei-su.github.io/)  
18. Youyi Zheng \- Github Stars \- Papers With Code, accessed on June 21, 2025, [https://paperswithcode.com/search?q=author%3AYouyi+Zheng\&order\_by=stars](https://paperswithcode.com/search?q=author:Youyi+Zheng&order_by=stars)  
19. ZJUGAPS-YYGroup \- GitHub, accessed on June 21, 2025, [https://github.com/ZJUGAPS-YYGroup](https://github.com/ZJUGAPS-YYGroup)  
20. libigl \- A simple C++ geometry processing library \- Computer Vision Group Jena, accessed on June 21, 2025, [https://git.inf-cv.uni-jena.de/3DFaces/libigl\_Martin/src/d568e6ce0dbd7c7d90239241d1c3938047d30882/README.md](https://git.inf-cv.uni-jena.de/3DFaces/libigl_Martin/src/d568e6ce0dbd7c7d90239241d1c3938047d30882/README.md)  
21. libigl tutorial, accessed on June 21, 2025, [https://libigl.github.io/tutorial/](https://libigl.github.io/tutorial/)  
22. Sparse matrix manipulations \- Eigen, accessed on June 21, 2025, [https://eigen.tuxfamily.org/dox/group\_\_TutorialSparse.html](https://eigen.tuxfamily.org/dox/group__TutorialSparse.html)  
23. libigl: Prototyping Geometry Processing Research in C++ \- Eurographics, accessed on June 21, 2025, [https://diglib.eg.org/bitstreams/0176cd55-f35c-40ba-921d-b40e672a753f/download](https://diglib.eg.org/bitstreams/0176cd55-f35c-40ba-921d-b40e672a753f/download)  
24. Geometric Computing with Python \- ResearchGate, accessed on June 21, 2025, [https://www.researchgate.net/profile/Francis-Williams-5/publication/334752129\_Geometric\_computing\_with\_python/links/5d4c29b94585153e5946741c/Geometric-computing-with-python.pdf](https://www.researchgate.net/profile/Francis-Williams-5/publication/334752129_Geometric_computing_with_python/links/5d4c29b94585153e5946741c/Geometric-computing-with-python.pdf)  
25. Chapter 1: Discrete Geometric Quantities and Operators \- libigl, accessed on June 21, 2025, [https://libigl.github.io/libigl-python-bindings/tut-chapter1/](https://libigl.github.io/libigl-python-bindings/tut-chapter1/)  
26. src/libigl/igl/harmonic.h · orthotropic-infill \- SCH \- GitLab, accessed on June 21, 2025, [https://git.sch.bme.hu/gyuri/prusaslicer/-/blob/orthotropic-infill/src/libigl/igl/harmonic.h](https://git.sch.bme.hu/gyuri/prusaslicer/-/blob/orthotropic-infill/src/libigl/igl/harmonic.h)  
27. Quick reference guide for sparse matrices \- Eigen, accessed on June 21, 2025, [https://eigen.tuxfamily.org/dox/group\_\_SparseQuickRefPage.html](https://eigen.tuxfamily.org/dox/group__SparseQuickRefPage.html)  
28. Sparse linear algebra \- Eigen, accessed on June 21, 2025, [https://eigen.tuxfamily.org/dox/group\_\_Sparse\_\_chapter.html](https://eigen.tuxfamily.org/dox/group__Sparse__chapter.html)  
29. Eigen.Solver.SparseLA \- Hackage, accessed on June 21, 2025, [https://hackage.haskell.org/package/eigen/docs/Eigen-Solver-SparseLA.html](https://hackage.haskell.org/package/eigen/docs/Eigen-Solver-SparseLA.html)  
30. libigl-python-bindings/tutorial/tutorials.ipynb at main \- GitHub, accessed on June 21, 2025, [https://github.com/libigl/libigl-python-bindings/blob/main/tutorial/tutorials.ipynb](https://github.com/libigl/libigl-python-bindings/blob/main/tutorial/tutorials.ipynb)  
31. Chapter 4: Parametrization \- igl \- libigl, accessed on June 21, 2025, [https://libigl.github.io/libigl-python-bindings/tut-chapter4/](https://libigl.github.io/libigl-python-bindings/tut-chapter4/)  
32. libigl/include/igl/harmonic.h at main \- GitHub, accessed on June 21, 2025, [https://github.com/libigl/libigl/blob/main/include/igl/harmonic.h](https://github.com/libigl/libigl/blob/main/include/igl/harmonic.h)  
33. Polygon Mesh Processing Reference \- CGAL manual, accessed on June 21, 2025, [https://doc.cgal.org/5.0/Polygon\_mesh\_processing/group\_\_PkgPolygonMeshProcessingRef.html](https://doc.cgal.org/5.0/Polygon_mesh_processing/group__PkgPolygonMeshProcessingRef.html)  
34. CGAL 4.8 \- Polygon Mesh Processing: User Manual, accessed on June 21, 2025, [https://doc.cgal.org/4.8/Polygon\_mesh\_processing/index.html](https://doc.cgal.org/4.8/Polygon_mesh_processing/index.html)  
35. Polygon Mesh Processing Reference \- CGAL manual, accessed on June 21, 2025, [https://doc.cgal.org/4.9/Polygon\_mesh\_processing/group\_\_PkgPolygonMeshProcessing.html](https://doc.cgal.org/4.9/Polygon_mesh_processing/group__PkgPolygonMeshProcessing.html)  
36. CGAL 4.10 \- Polygon Mesh Processing: User Manual, accessed on June 21, 2025, [https://doc.cgal.org/4.10/Polygon\_mesh\_processing/index.html](https://doc.cgal.org/4.10/Polygon_mesh_processing/index.html)  
37. Reconstruct all the edges in a triangle mesh \- Stack Overflow, accessed on June 21, 2025, [https://stackoverflow.com/questions/22204673/reconstruct-all-the-edges-in-a-triangle-mesh](https://stackoverflow.com/questions/22204673/reconstruct-all-the-edges-in-a-triangle-mesh)  
38. Vector Heat Method \- Geometry Central, accessed on June 21, 2025, [https://geometry-central.net/surface/algorithms/vector\_heat\_method/](https://geometry-central.net/surface/algorithms/vector_heat_method/)  
39. Signed Heat Method \- Geometry Central, accessed on June 21, 2025, [https://geometry-central.net/surface/algorithms/signed\_heat\_method/](https://geometry-central.net/surface/algorithms/signed_heat_method/)  
40. Polygon Mesh Heat Distance & Transport \- Geometry Central, accessed on June 21, 2025, [https://geometry-central.net/surface/algorithms/polygon\_heat\_solver/](https://geometry-central.net/surface/algorithms/polygon_heat_solver/)  
41. Geodesic Distance \- Geometry Central, accessed on June 21, 2025, [https://geometry-central.net/surface/algorithms/geodesic\_distance/](https://geometry-central.net/surface/algorithms/geodesic_distance/)  
42. geodesic \- ExactGeodesic.wiki \- Google Code, accessed on June 21, 2025, [https://code.google.com/archive/p/geodesic/wikis/ExactGeodesic.wiki](https://code.google.com/archive/p/geodesic/wikis/ExactGeodesic.wiki)  
43. dfsp-spirit/cpp\_geodesics: Fast computation of geodesic distances and related mesh descriptors on (brain surface) meshes in C++ and OpenMP. \- GitHub, accessed on June 21, 2025, [https://github.com/dfsp-spirit/cpp\_geodesics](https://github.com/dfsp-spirit/cpp_geodesics)  
44. the-virtual-brain/tvb-gdist: Geodesic Library (adaptation by TVB Team) \- GitHub, accessed on June 21, 2025, [https://github.com/the-virtual-brain/tvb-gdist](https://github.com/the-virtual-brain/tvb-gdist)  
45. Geodesic Distance Computation via Virtual Source Propagation, accessed on June 21, 2025, [https://www.graphics.rwth-aachen.de/publication/03334/](https://www.graphics.rwth-aachen.de/publication/03334/)  
46. 3DFaces/libigl\_Martin: Extension of libigl which allows to also read the texture of wrl-files in Python as igl.read\_triangle\_mesh(wrFilePath, V, F, TC) @ ecbd68afb2dc8339130ee31b6350861bcc515e2e \- Computer Vision Group Jena, accessed on June 21, 2025, [https://git.inf-cv.uni-jena.de/3DFaces/libigl\_Martin/src/ecbd68afb2dc8339130ee31b6350861bcc515e2e/tutorial/readme.md](https://git.inf-cv.uni-jena.de/3DFaces/libigl_Martin/src/ecbd68afb2dc8339130ee31b6350861bcc515e2e/tutorial/readme.md)  
47. cotmatrix\_entries return half of expected values · Issue \#382 · libigl/libigl \- GitHub, accessed on June 21, 2025, [https://github.com/libigl/libigl/issues/382](https://github.com/libigl/libigl/issues/382)  
48. eigen3/doc/C09\_TutorialSparse.dox at master \- GitHub, accessed on June 21, 2025, [https://github.com/OPM/eigen3/blob/master/doc/C09\_TutorialSparse.dox](https://github.com/OPM/eigen3/blob/master/doc/C09_TutorialSparse.dox)  
49. sparse matrix from triplet \- c++ \- Stack Overflow, accessed on June 21, 2025, [https://stackoverflow.com/questions/29830299/sparse-matrix-from-triplet](https://stackoverflow.com/questions/29830299/sparse-matrix-from-triplet)  
50. Eigen::Triplet\< Scalar, StorageIndex \> Class Template Reference, accessed on June 21, 2025, [https://eigen.tuxfamily.org/dox/classEigen\_1\_1Triplet.html](https://eigen.tuxfamily.org/dox/classEigen_1_1Triplet.html)  
51. Marching squares \- Wikipedia, accessed on June 21, 2025, [https://en.wikipedia.org/wiki/Marching\_squares](https://en.wikipedia.org/wiki/Marching_squares)  
52. 3D Surface Mesh Generation: CGAL::Gray\_level\_image\_3\< FT\_, Point\_ \> Class Template Reference, accessed on June 21, 2025, [https://doc.cgal.org/latest/Surface\_mesher/classCGAL\_1\_1Gray\_\_level\_\_image\_\_3.html](https://doc.cgal.org/latest/Surface_mesher/classCGAL_1_1Gray__level__image__3.html)  
53. Classification/gis\_tutorial\_example.cpp \- CGAL 6.0.1 \- Manual, accessed on June 21, 2025, [https://doc.cgal.org/latest/Manual/Classification\_2gis\_tutorial\_example\_8cpp-example.html](https://doc.cgal.org/latest/Manual/Classification_2gis_tutorial_example_8cpp-example.html)  
54. CGAL 5.1 \- Manual: GIS (Geographic Information System) \- Inria, accessed on June 21, 2025, [https://geometrica.saclay.inria.fr/team/Marc.Glisse/tmp/cgal-power/Manual/tuto\_gis.html](https://geometrica.saclay.inria.fr/team/Marc.Glisse/tmp/cgal-power/Manual/tuto_gis.html)  
55. GIS (Geographic Information System) \- CGAL 6.0.1 \- Manual, accessed on June 21, 2025, [https://doc.cgal.org/latest/Manual/tuto\_gis.html](https://doc.cgal.org/latest/Manual/tuto_gis.html)  
56. tutorial, accessed on June 21, 2025, [https://zishun.github.io/projects/MeshUtility/](https://zishun.github.io/projects/MeshUtility/)  
57. C++ code for cotangent weights over a triangular mesh \- Rodolphe Vaillant's homepage, accessed on June 21, 2025, [http://mobile.rodolphe-vaillant.fr/?e=69](http://mobile.rodolphe-vaillant.fr/?e=69)