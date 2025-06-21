Test plan for improved segmentation system:

1. Load man0.off mesh
2. Run Enhanced Rigidity Analysis (option 10) - this will now:
   - Generate skeletal segments from MULTIPLE FPS point pairs (not just 2)
   - Create many more skeletal nodes for better rigidity analysis
   - Should give a much wider range of rigidity values

3. Use Interactive Rigidity Threshold Testing (option 12) to find optimal threshold:
   - Try thresholds around 0.9 (paper recommendation)
   - Look for threshold that gives ~4-8 junction points for human model
   - Should see meaningful variation now with more skeletal nodes

Expected improvements:
- Many more skeletal nodes (50+ instead of 4)
- Better rigidity value distribution
- Meaningful threshold sensitivity
- Clear junction point detection at shoulders, hips, neck, etc.
