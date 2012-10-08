============== Description =======================
generate_hypothesis_node takes a pointcloud with floor aligned to XY plane and generates a message of 2 arrays - poses and class names.

fit_models_node takes these arrays and fits models and publishes fitted models as pointcloud.


================ Instalations =====================
Follow the instalation instructions for fuerte on http://www.ros.org/wiki/furniture_classification
after that go to the data/ directory in furniture_classification and run train_classifier.sh. It will create partial scans and train the database.
It will save the database to data/database/. It contains training result, parameters and mesh models. Every node requires a path to the database
as one of the parameters

============== Running ====================
To start classification run: 

roslaunch furniture_classification classification.launch

To view results run (write full path to view_nodes.vcg if you are not in furniture_classification directory):

rosrun rviz rviz -d view_nodes.vcg


