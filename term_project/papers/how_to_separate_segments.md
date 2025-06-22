Excellent question. You've hit on the most critical and nuanced step of this entire process: moving from a collection of limb-like skeletal segments to a complete, meaningful segmentation that includes the "main parts" or torso.

Your intuition is exactly correct: **the low-rigidity points are the key.**

Let's break this down based on the paper's methodology and general practices in the field.

### The Core Concept: Defining Parts by Rigidity

First, let's revisit the core idea of rigidity from the paper:

* **High Rigidity:** These are points on "thin" or "tubular" parts of a model. The harmonic isocurves around these points are tightly constrained, like circles. Think of arms, legs, fingers, or the armadillo's tail. The algorithm is excellent at finding skeletal segments that run along these high-rigidity regions. A limb segment, like from a hand to a shoulder, is essentially a path of high rigidity.
* **Low Rigidity:** These are points on "thick," "volumetric," or "junction" areas. The isocurves are less constrained and can spread out. This perfectly describes a torso, the junction where legs meet the body (hips), or where arms meet the body (shoulders).

Therefore, the researchers define the main connecting parts (the torso) **implicitly**. It is the region of the shape that is **left over** after all the high-rigidity limb segments have been identified.

### The Algorithmic Pipeline to Find the Torso

Here is the step-by-step process for how a human model would be segmented, leading to the identification of the torso.

**Step 1: Compute Rigidity and Find Extremities**
First, the algorithm computes the rigidity value for every point on the mesh. It also identifies the extremity points (hands, feet, head).

**Step 2: Identify High-Rigidity "Limb" Segments**
The algorithm then finds the optimal skeletal paths connecting pairs of extremities that stay within high-rigidity zones. For a human model, it would identify segments like:

* `Right Hand <---> Left Hand`
* `Right Hand <---> Right Foot`
* `Head <---> Right Hand`
* etc.

The paper's "Simultaneous Skeletonization and Segmentation" algorithm (Section 3.3) selects the most "dominant" of these skeletal paths. For a human, these would correspond to the four limbs and the path from the torso to the head. The segment for a limb should ideally start at the extremity (e.g., hand) and end where the rigidity drops significantly (e.g., the shoulder).

**Step 3: Initial "Limb" Segmentation (The Key Step)**
Now, the algorithm performs an initial segmentation. For every vertex `p` on the mesh, it associates `p` with the skeletal segment that "explains it best." In simple terms, each vertex is assigned to the nearest or most influential skeletal segment.

This step successfully carves out the obvious parts:
* All vertices near the "right arm" skeleton get the "Right Arm" label.
* All vertices near the "left leg" skeleton get the "Left Leg" label.
* ...and so on.

**Step 4: Grouping the Remainder to Form the Torso**

This is the answer to your question. After you've assigned all the high-confidence vertices to their respective limbs, what are you left with?

You are left with a large collection of unassigned vertices. These vertices were not strongly claimed by the arm skeleton *or* the leg skeleton because they are in the middle. **These are precisely the low-rigidity points that constitute the torso.**

So, the torso is defined as the set of all vertices that were not assigned to a primary limb segment in Step 3. You can then do the following:

* **Simple Approach:** Assign all of these leftover vertices to a single new segment called "Torso."
* **Refined Approach:** Sometimes, the "leftovers" might include the torso and the head as separate, disconnected blobs of vertices. To handle this, you run a **Connected Components Analysis** on the leftover vertices. This algorithm will find all the disconnected groups. The largest group is your torso. Other smaller groups would be the neck or head, which you can label accordingly.

This is exactly what you hypothesized: **"low rigidity points are handled together maybe finding centroid of that part and clustering together to show segment."** That's it! The cluster is simply "all the points that don't belong to a limb."

### Your Armadillo Example

Your analysis of the armadillo skeleton is spot on. The five polylines are the five most dominant skeletal paths the algorithm found.
* **Pink (Hand to Hand):** This captures the pectoral/shoulder region.
* **Green/Yellow/Red/Blue:** These capture the relationships between the other extremities (legs, head).

The final segmentation would work as follows:
1.  Vertices near the pink polyline are assigned to the arms/front.
2.  Vertices near the other polylines are assigned to their respective leg/head regions.
3.  The vertices in the main shell of the armadillo have low rigidity and are not close to any single one of those extremity-to-extremity paths. They are the "leftovers" and are grouped together to form the "torso" or "body" segment.

### Studies to Check (Keywords for Your Research)

To find more papers on this, here are the terms researchers use. You were on the right track.

* **"Shape Diameter Function (SDF)":** This is a classic and very popular alternative to the paper's "rigidity" function. It also measures the "thickness" of a part and is often used to distinguish torsos from limbs. Many papers use SDF for a first-pass segmentation.
* **"Skeleton-based Segmentation":** This is the general term for the family of techniques you are exploring.
* **"Part-based Segmentation" / "Semantic Segmentation":** These terms describe the higher-level goal of not just cutting a mesh into pieces, but cutting it into meaningful parts like "arm," "leg," and "torso."
* **"Fuzzy Clustering" / "Soft Segmentation":** These methods, like the one in the paper, calculate a "probability" that a point belongs to each segment, rather than making a hard decision. The low-rigidity torso points would have low probabilities for all limb segments.