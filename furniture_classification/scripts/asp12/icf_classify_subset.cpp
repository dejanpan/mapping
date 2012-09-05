#include <icf_core/client/Client.h>
#include <icf_core/base/EvaluationResult.hpp>
#include <icf_core/base/ConfusionMatrix.hpp>
#include <icf_core/base/ClassificationResult.hpp>

...

  std::map<int, std::string> classnames;
  std::string classifier;
  std::string parameters;

  Eigen::MatrixXf conf_mat;

  // TODO: use boost_shared_ptr
  ClassifierManager* g_manager;
  ServerSideRepo* data_store;  
  ClassifierClient* client_test;
  
  ~Destructor ()
  {
    // delete pointers
    delete g_manager; 
    delete data_store;
    delete client_test;
  }

  Constructor()
  {
    // use here the same things as in "icf_train_subset.sh" of course, but with KNN you can change some stuff as well (-k, -w for sure)
    classifier = "knn";
    parameters = "-m chisquared -k 10 -w";
    
    // maybe this needs ~ expanded, just locate it and pass it in as a parameter
    std::string model_file_base = "~/.ros/vfh_subsets1to3_with_evaluation_knn"; // NOTE: there are multiple files with this base name
    
    classnames[1]="Armchair";
    classnames[2]="Chair1";
    classnames[3]="Chair2";
    classnames[4]="Chair3";
    classnames[5]="Sideboard";
    classnames[6]="Table1";
    classnames[7]="Table2";
    classnames[8]="Table3";
    // etc.

    // init data uploading and client
    std::string manager_name = "kinect_uima_bridge";
    data_store = new ServerSideRepo(n, manager_name);
    client_test = new ClassifierClient(n, manager_name, classifier, parameters);
    client_test->load(model_file_base);
    
    // optional, use confusion matrix to weight we resulting scores (i.e. accuracy weighting)
    conf_mat = client_test->getConfusionMatrix()->getCM().cast<float>();
    Eigen::ArrayXf col_sums = conf_mat.colwise().sum();
    //for (int i=0; i<col_sums.size(); ++i)
    //  conf_mat.row(i) = conf_mat.row(i).array() / col_sums;
    for (int i=0; i<conf_mat.rows(); ++i)
      for (int j=0; j<conf_mat.cols(); ++j)
        conf_mat(i,j) /= col_sums(j);
    //std::cerr << conf_mat << std::endl;
  }

  void classify (std::vector<float> feature) // or any other way to input the feature vector to be classified
  {
    try
    {  
      // set up dataset
      DS ds;
      DS::Matrix feature_matrix(1, feature.size());
      feature_matrix.row(0) = Eigen::VectorXf::Map(&feature[0], feature.size()).cast<double>(); // or any other way to get the feature
      ds.setFeatureMatrix(feature_matrix, "/x");
      //ds.setFeatureMatrix(&feature[0], 1, feature.size(), "/test"); needs double!
      data_store->uploadData(ds, "test");
      
      // classify data
      client_test->assignData("test", icf::Classify);
      ClassificationResult classificationResult = client_test->classify();
      
      // best result
      int result = classificationResult.results->at(0); // here we classify a single feaure vector, but we could do multiple at once
      
      // confidences and accuracies for all classes
      for(std::map<int,std::string>::iterator mit = classnames.begin(); mit != classnames.end(); ++mit)
      {
        // example use assuming a numbering of classes from 1
        float accuracy = conf_mat(mit->first-1,result-1); // TODO add pseudocounts to conf matrix or a minimum probability
        float confidence = classificationResult.confidenceFor(0,mit->first); // if k=1 this will be 1 for mit->first==result and 0 for all others classes
        float score = confidence * accuracy; // or one of them left out :)
      }
    }
    catch (ICFException& e)
    {
      std::cerr << boost::diagnostic_information(e);
      std::vector<service_unavailable_error>* err = boost::get_error_info<service_unavailable_collection>(e);
      if (err != NULL)
      {
        for (std::vector<service_unavailable_error>::iterator iter = err->begin(); iter != err->end(); iter++)
          std::cerr << "Error: " << iter->value() << std::endl;
      }
      else
        std::cerr << "No service availability related errors" << std::endl;
    }
  }

