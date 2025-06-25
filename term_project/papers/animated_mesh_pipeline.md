The skeleton will define the "parts," and the CRF will create a high-quality segmentation that cleanly assigns the mesh surface to those skeletal parts.

Here is a clear, step-by-step explanation of the full implementation pipeline.

### Systematic Pipeline: Skeleton-Driven CRF for Animated Meshes

The entire process can be broken into two main stages. Stage 1 is the slow, heavy analysis that you only run **once** on the clean reference mesh. Stage 2 is the fast, lightweight loop that you run for every frame of the animation to create the final video.

---

### **Stage 1: Static Analysis (Run Once)**

This stage generates all the core data from the single reference mesh (e.g., `horse-gallop-reference.off`).

#### **Step 1.A: Generate the Static Skeleton**

* **Input:** The single reference mesh.
* **Process:** Run your existing skeleton generation algorithm (the one based on Pairwise Harmonics that produces `rigidSegments`). This defines the fundamental articulated structure of the character.
* **Output:** A **Static Skeleton**, which is a list of joints (the endpoints) and bones (the segments connecting them).

#### **Step 1.B: Segment the Surface with Skeleton-Driven CRF**

* **Input:**
    1.  The reference mesh.
    2.  The **Static Skeleton** from Step 1.A.
* **Process:** This is where you use your CRF. The bones of the skeleton become the "labels" or "terminals" for the segmentation.
    1.  **Define Data Cost:** For each face on the mesh, calculate its "cost" to belong to each *bone* of the skeleton. This cost can be based on harmonic values or simple Euclidean distance to the bone, just as you planned before.
    2.  **Define Smoothness Cost:** Calculate the cost of having a segment boundary between any two adjacent faces (e.g., using the dihedral angle).
    3.  Run your CRF solver (graph cut) to find the optimal assignment of each face to a skeleton bone.
* **Output:** A **Master Segmentation**, which is a high-quality, per-vertex (or per-face) set of labels that perfectly aligns with your generated skeleton.

#### **Step 1.C: Bind the Skeleton for Deformation**

* **Input:**
    1.  The **Static Skeleton** from Step 1.A.
    2.  The surface of the reference mesh.
* **Process:** Attach each joint of the skeleton to the mesh surface using **barycentric coordinates**. For each joint, find the closest triangle on the mesh and save its `(u, v, w)` coordinates relative to that triangle's vertices.
* **Output:** A list of **Binding Data** (e.g., `[joint_0, triangle_12, (0.5, 0.3, 0.2)]`, `[joint_1, triangle_58, (0.1, 0.8, 0.1)]`, etc.).

---

### **Stage 2: Dynamic Propagation (A Fast Loop for Every Frame)**

Now you loop through every frame of the animation (e.g., `horse-gallop-01.off` to `horse-gallop-48.off`). All the steps inside this loop are extremely fast as they use the pre-computed data from Stage 1.

#### **Step 2.A: Transfer the Segmentation**

* **Input:** The **Master Segmentation** labels from Step 1.B.
* **Process:** Apply the labels directly to the current animation frame using the one-to-one vertex correspondence.
* **Output:** The current frame of the horse, now colored with a stable, flicker-free segmentation.

#### **Step 2.B: Deform the Skeleton**

* **Input:** The **Binding Data** from Step 1.C.
* **Process:** Use the saved barycentric coordinates to calculate the new 3D position of each joint based on the new positions of the vertices of its bound triangle in the current animation frame.
* **Output:** A **Deformed Skeleton** for the current frame, perfectly posed with the mesh.

#### **Step 2.C: Render the Final Frame**

* **Input:**
    1.  The colored mesh from Step 2.A.
    2.  The deformed skeleton from Step 2.B.
* **Process:** Render the image for the current frame, drawing the deformed skeleton inside the colored mesh. Save the image.
* **Output:** One frame of your final video.

After looping through all animation frames, you can combine the rendered images to create your final video deliverable.