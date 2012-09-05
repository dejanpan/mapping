c) speed-up as a result of the classification step before the model
fitting step.
1) for this we need some training data, based on the CAD models
2) I took the scans Vlad generated (which contain the segmentation results and correctly flipped normals)
3) running the script "break_parts.sh" in the folder with the PCD files will generate a folder structure and place the clusters/parts obtained from segmentation as separate PCD files: <label>_<name>_<id>/scan<nr>/cluster<count>.pcd
3) then the script "reconstruct_parts.sh" in the same location will generate random groupings of these parts, with each new PCD containing 1 to 3 parts (because during detection that is usually how many we get): <label>_<name>_<id>/all<nr_parts>_<contained_clusters>_scan<nr>_subset.pcd
4) now we have to extract some feature (VFH in our case) for these training examples, build a classifier (SVM, KNN using any FLANN metric or AdaBoost for example), and save the model, using the script "icf_train_subset.sh": first param is the base folder for the ones in 3), second is the KNN distance metric, I used L2 for the evaluation in the paper, but maybe "chisquared" should be specified instead as Radu published it, and the third parameter is the value for k, set it to >1 if you need some ranking, not just "the best"/one CAD model (don't forget to "roslaunch icf_core icf_service_node.launch", as indicated by a comment in the file)

NOTES:
- ICF (IAS Classification Framework): http://code.in.tum.de/indefero/index.php/p/ias-cf/
- in the paper we evaluated the classifier using 5 fold cross validation (by leaving out 20% of scans of a CAD model at random), had some extra scripts for these (nothing interesting, we should focus on the evaluation of the real scans)
- if you provide "random" as parameter to "reconstruct_parts.sh", it will just create a max of 100 PCDs out of each grouping type (<nr_parts>), at random: useful to get rid of the exponential growth of number of training examples as <nr_parts> increases (will test its effect on the real scans by creating separate models with different max numbers of PCDs)



- how to run VFH classification and generate the confusion matrix.

1) after a local maxima is detected and the corresponding parts extracted, the previously built classifier needs to be run in order to reduce the list of CAD models that need to be fitted (either to a fixed number, to those having a score above a threshold, or a combination)
2) this can be done using the example code snippet in "icf_classify_subset.cpp", has to depend on "icf_core" and link against "icf_client"
3) either run a classifier manager using the launch file (as for "icf_train_subset.sh") or you can link against "icf_service_lib" as well to spin off a thread and run it in your executable (as in "icf_core/examples/example_client.cpp")

NOTE:
- maybe providing some negative examples as an extra "background" class would help, generated from false positive detections from processed scans (simplest method: run the algorithm on PCDs not containing any object we want to detect and then every detection is a false positive example)


LOW PRIORITY, just for future reference
and one more: how to generate the link constraints.

we did not try if link constraints help RIM, but we could, by:
1) Vlad sending a txt file with the SGF features for each cluster to Flo (you sent this already to me, so this is done)
2) Vlad saving each cluster to a separate file during training in a specific folder structure: <cad_model>/scan<nr>/cluster<count>.pcd (preferably the same as "break_parts.sh" above, thus making that script obsolete)
3) Vlad sending a second txt file with a specific string for each cluster to Flo, same ordering as in the first file of course (you sent something like this already to me, containing the CAD model name, so should be easy to change): <cad_model>_scan<nr>_cluster<count>.pcd (so the same as the path, just '_' instead of '/', or just put in that relative path and we can text replace)
4) in the base folder of the generated folder structure from 2) run the script "reconstruct_part_links.sh" to generate a third txt file holding the list of cluster pairs that have high overlap on the CAD model (currently runs only on my machine, requires the old _Clouds repo, but the code is easy to port)
5) Flo takes the 3 txt files and runs C-RIM to generate a hopefully better model than simple RIM

NOTEs:
- obviously 1)-3) can be done at the same step, making it simple and reduce the possibility of bugs :)
- we were able to do this (circumventing some steps) using VFH, but it is too high-dimensional to work nicely, hopefully with SGF it will work better (or we try dimensionality reduction on VFH)
