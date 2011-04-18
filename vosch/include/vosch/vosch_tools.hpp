//-----------
//* read
template <typename T>
bool readPoints( const char *name, pcl::PointCloud<T>& cloud ){
  if (pcl::io::loadPCDFile (name, cloud) == -1){
    ROS_ERROR ("Couldn't read file %s",name);
    return (-1);
  }
#ifndef QUIET
  ROS_INFO ("Loaded %d data points from %s with the following fields: %s", (int)(cloud.width * cloud.height), name, pcl::getFieldsList (cloud).c_str ());
#endif
  return(1);
}

//------------------
//* compute normals
template <typename T1, typename T2>
void computeNormal( pcl::PointCloud<T1> input_cloud, pcl::PointCloud<T2>& output_cloud ){
  // if ((int)input_cloud.points.size () < k_)
  // {
  //   ROS_WARN ("Filtering returned %d points! Continuing.", (int)input_cloud.points.size ());
  //   //pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_temp;
  //   //pcl::concatenateFields <Point, pcl::Normal, pcl::PointXYZRGBNormal> (cloud_filtered, cloud_normals, cloud_temp);
  //   //pcl::io::savePCDFile ("test.pcd", cloud, false);
  // }
  pcl::NormalEstimation<T1, T2> n3d_;
  //n3d_.setKSearch (k_);
  n3d_.setRadiusSearch (normals_radius_search);
  n3d_.setSearchMethod ( boost::make_shared<pcl::KdTreeFLANN<T1> > () );
  n3d_.setInputCloud ( input_cloud.makeShared() );
  n3d_.compute (output_cloud);

  //* TODO: move the following lines to NormalEstimation class ?
  for ( int i=0; i< (int)input_cloud.points.size (); i++ ){
    output_cloud.points[ i ].x = input_cloud.points[ i ].x;
    output_cloud.points[ i ].y = input_cloud.points[ i ].y;
    output_cloud.points[ i ].z = input_cloud.points[ i ].z;
    output_cloud.points[ i ].rgb = input_cloud.points[ i ].rgb;
  }
}

//--------------------
//* function for GRSD 
int getType (float min_radius, float max_radius)
{
  if (min_radius > 0.100)
    return PLANE;
  else if (max_radius > 0.175)
    return CYLINDER;
  else if (min_radius < 0.015)
    return NOISE;
  else if (max_radius - min_radius < 0.050)
    return SPHERE;
  else
    return EDGE;
  /* if (min_radius > min_radius_plane_) // 0.066 */
  /*   return PLANE; // plane */
  /* else if ((min_radius < min_radius_noise_) && (max_radius < max_radius_noise_)) */
  /*   return NOISE; // noise/corner */
  /* else if (max_radius - min_radius < max_min_radius_diff_) // 0.0075 */
  /*   return SPHERE; // circle (corner?) */
  /* else if (min_radius < min_radius_edge_) /// considering small cylinders to be edges */
  /*   return EDGE; // edge */
  /* else */
  /*   return CYLINDER; // cylinder (rim) */
}

//--------------------
//* extract - GRSD -
template <typename T>
Eigen::Vector3i extractGRSDSignature21(pcl::VoxelGrid<T> grid, pcl::PointCloud<T> cloud, pcl::PointCloud<T> cloud_downsampled, std::vector< std::vector<float> > &feature, const float voxel_size, const int subdivision_size = 0, const int offset_x = 0, const int offset_y = 0, const int offset_z = 0, const bool is_normalize  ){
#ifndef QUIET
  ROS_INFO("rsd %f, normals %f, leaf %f", rsd_radius_search, normals_radius_search, voxel_size);
#endif
  feature.resize( 0 );
  boost::shared_ptr< const pcl::PointCloud<T> > cloud_downsampled_ptr;
  cloud_downsampled_ptr.reset (new pcl::PointCloud<T> (cloud_downsampled));

  //* for computing multiple GRSD with subdivisions
  int hist_num = 1;
  float inverse_subdivision_size = 0;
  Eigen::Vector3i div_b_= Eigen::Vector3i::Zero();
  Eigen::Vector3i min_b_= Eigen::Vector3i::Zero();
  Eigen::Vector3i subdiv_b_ = Eigen::Vector3i::Zero();
  Eigen::Vector3i subdivb_mul_= Eigen::Vector3i::Zero();
  if( subdivision_size > 0 ){
    inverse_subdivision_size = 1.0 / subdivision_size;
    div_b_ = grid.getNrDivisions();
    min_b_ = grid.getMinBoxCoordinates();
    if( ( div_b_[0] <= offset_x ) || ( div_b_[1] <= offset_y ) || ( div_b_[2] <= offset_z ) ){
      std::cerr << "(In extractGRSDSignature21) offset values (" << offset_x << "," << offset_y << "," << offset_z << ") exceed voxel grid size (" << div_b_[0] << "," << div_b_[1] << "," << div_b_[2] << ")."<< std::endl;
      return Eigen::Vector3i::Zero();
    }
    subdiv_b_ = Eigen::Vector3i ( ceil( ( div_b_[0] - offset_x )*inverse_subdivision_size ), ceil( ( div_b_[1] - offset_y )*inverse_subdivision_size ), ceil( ( div_b_[2] - offset_z )*inverse_subdivision_size ) );
    subdivb_mul_ = Eigen::Vector3i ( 1, subdiv_b_[0], subdiv_b_[0] * subdiv_b_[1] );
    hist_num = subdiv_b_[0] * subdiv_b_[1] * subdiv_b_[2];
  }
  else if( subdivision_size < 0 ){
    std::cerr << "(In extractGRSDSignature21) Invalid subdivision size: " << subdivision_size << std::endl;
    return Eigen::Vector3i::Zero();
  }

  // Compute RSD
  pcl::RSDEstimation <T, T, pcl::PrincipalRadiiRSD> rsd;
  rsd.setInputCloud( cloud_downsampled_ptr );
  boost::shared_ptr< const pcl::PointCloud<T> > cloud_ptr = cloud.makeShared();
  rsd.setSearchSurface( cloud_ptr );
  rsd.setInputNormals( cloud_ptr );
#ifndef QUIET
  ROS_INFO("radius search: %f", std::max(rsd_radius_search, voxel_size/2 * sqrt(3)));
#endif
  rsd.setRadiusSearch(std::max(rsd_radius_search, voxel_size/2 * sqrt(3)));
  boost::shared_ptr< pcl::KdTree<T> > tree2 = boost::make_shared<pcl::KdTreeFLANN<T> > ();
  tree2->setInputCloud (cloud.makeShared());
  rsd.setSearchMethod(tree2);
  pcl::PointCloud<pcl::PrincipalRadiiRSD> radii;
  t1 = my_clock();
  rsd.compute(radii);
#ifndef QUIET
  ROS_INFO("RSD compute done in %f seconds.", my_clock()-t1);
#endif
  
  // Get rmin/rmax for adjacent 27 voxel
  t1 = my_clock();
  Eigen::MatrixXi relative_coordinates (3, 13);

  //Eigen::MatrixXi transition_matrix =  Eigen::MatrixXi::Zero(6, 6);
  std::vector< Eigen::MatrixXi > transition_matrix( hist_num );
  for( int i=0; i<hist_num; i++ )
    transition_matrix[ i ] =  Eigen::MatrixXi::Zero(6, 6);

  int idx = 0;
  
  // 0 - 8
  for( int i=-1; i<2; i++ )
  {
    for( int j=-1; j<2; j++ )
    {
      relative_coordinates( 0, idx ) = i;
      relative_coordinates( 1, idx ) = j;
      relative_coordinates( 2, idx ) = -1;
      idx++;
    }
  }
  // 9 - 11
  for( int i=-1; i<2; i++ )
  {
    relative_coordinates( 0, idx ) = i;
    relative_coordinates( 1, idx ) = -1;
    relative_coordinates( 2, idx ) = 0;
    idx++;
  }
  // 12
  relative_coordinates( 0, idx ) = -1;
  relative_coordinates( 1, idx ) = 0;
  relative_coordinates( 2, idx ) = 0;

  Eigen::MatrixXi relative_coordinates_all (3, 26);
  relative_coordinates_all.block<3, 13>(0, 0) = relative_coordinates;
  relative_coordinates_all.block<3, 13>(0, 13) = -relative_coordinates;
  
  // Get transition matrix
  std::vector<int> types (radii.points.size());
 
  for (size_t idx = 0; idx < radii.points.size (); ++idx)
    types[idx] = getType(radii.points[idx].r_min, radii.points[idx].r_max);
  
  for (size_t idx = 0; idx < cloud_downsampled_ptr->points.size (); ++idx)
  {
    // calc hist_idx
    int hist_idx;
    if( hist_num == 1 ) hist_idx = 0;
    else{
      const int tmp_x = floor( cloud_downsampled_ptr->points[ idx ].x/voxel_size ) - min_b_[ 0 ] - offset_x;
      const int tmp_y = floor( cloud_downsampled_ptr->points[ idx ].y/voxel_size ) - min_b_[ 1 ] - offset_y;
      const int tmp_z = floor( cloud_downsampled_ptr->points[ idx ].z/voxel_size ) - min_b_[ 2 ] - offset_z;
      /* const int x_mul_y = div_b_[0] * div_b_[1]; */
      /* const int tmp_z = idx / x_mul_y - offset_z; */
      /* const int tmp_y = ( idx % x_mul_y ) / div_b_[0] - offset_y; */
      /* const int tmp_x = idx % div_b_[0] - offset_x; */
      if( ( tmp_x < 0 ) || ( tmp_y < 0 ) || ( tmp_z < 0 ) ) continue; // ignore idx smaller than offset.
      Eigen::Vector3i ijk = Eigen::Vector3i ( floor ( tmp_x * inverse_subdivision_size), floor ( tmp_y * inverse_subdivision_size), floor ( tmp_z * inverse_subdivision_size) );
      hist_idx = ijk.dot (subdivb_mul_);
    }

    int source_type = types[idx];
    std::vector<int> neighbors = grid.getNeighborCentroidIndices ( cloud_downsampled_ptr->points[idx], relative_coordinates_all);
    for (unsigned id_n = 0; id_n < neighbors.size(); id_n++)
    {
      int neighbor_type;
      if (neighbors[id_n] == -1)
        neighbor_type = EMPTY;
      else
        neighbor_type = types[neighbors[id_n]];

      transition_matrix[ hist_idx ](source_type, neighbor_type)++;
    }
  }
#ifndef QUIET
  std::cerr << "transition matrix" << std::endl << transition_matrix[0] << std::endl;
  ROS_INFO("GRSD compute done in %f seconds.", my_clock()-t1);
#endif

  pcl::PointCloud<pcl::GRSDSignature21> cloud_grsd;
  cloud_grsd.points.resize(hist_num);
  
  for( int h=0; h<hist_num; h++ ){
    int nrf = 0;
    for (int i=0; i<NR_CLASS+1; i++)
      for (int j=i; j<NR_CLASS+1; j++)
    /* for (int i=1; i<NR_CLASS+1; i++) */
    /*   for (int j=0; j<=i; j++) */
    cloud_grsd.points[ h ].histogram[nrf++] = transition_matrix[ h ](i, j); //@TODO: resize point cloud
  }

  feature.resize( hist_num );
  if( is_normalize ){
    for( int h=0; h<hist_num; h++ ){
      feature[ h ].resize( 20 );
      for( int i=0; i<20; i++)
        feature[ h ][ i ] = cloud_grsd.points[ h ].histogram[ i ] * NORMALIZE_GRSD;
    }
  }
  else{
    for( int h=0; h<hist_num; h++ ){
      feature[ h ].resize( 20 );
      for( int i=0; i<20; i++)
        feature[ h ][ i ] = cloud_grsd.points[ h ].histogram[ i ];
    }
  }
  return subdiv_b_;
}

template <typename T>
void extractGRSDSignature21(pcl::VoxelGrid<T> grid, pcl::PointCloud<T> cloud, pcl::PointCloud<T> cloud_downsampled, std::vector<float> &feature, const float voxel_size, const bool is_normalize  ){
  std::vector< std::vector<float> > tmp( 1 );
  extractGRSDSignature21( grid, cloud, cloud_downsampled, tmp, voxel_size, 0, 0, 0, 0, is_normalize ); // for one signature
  feature = tmp[ 0 ];
}

//-----------------------------------
//* extract - rotation-variant GRSD -
template <typename T>
Eigen::Vector3i extractGRSDSignature325(pcl::VoxelGrid<T> grid, pcl::PointCloud<T> cloud, pcl::PointCloud<T> cloud_downsampled, std::vector< std::vector<float> > &feature, const float voxel_size, const int subdivision_size = 0, const int offset_x = 0, const int offset_y = 0, const int offset_z = 0, const bool is_normalize  ){
#ifndef QUIET
  ROS_INFO("rsd %f, normals %f, leaf %f", rsd_radius_search, normals_radius_search, voxel_size);
#endif
  feature.resize( 0 );
  boost::shared_ptr< const pcl::PointCloud<T> > cloud_downsampled_ptr;
  cloud_downsampled_ptr.reset (new pcl::PointCloud<T> (cloud_downsampled));
  pcl::PointCloud<pcl::GRSDSignature325> cloud_grsd;
  
  //* for computing multiple GRSD with subdivisions
  int hist_num = 1;
  float inverse_subdivision_size = 0;
  Eigen::Vector3i div_b_= Eigen::Vector3i::Zero();
  Eigen::Vector3i min_b_= Eigen::Vector3i::Zero();
  Eigen::Vector3i subdiv_b_= Eigen::Vector3i::Zero();
  Eigen::Vector3i subdivb_mul_= Eigen::Vector3i::Zero();
  if( subdivision_size > 0 ){
    inverse_subdivision_size = 1.0 / subdivision_size;
    div_b_ = grid.getNrDivisions();
    min_b_ = grid.getMinBoxCoordinates();
    if( ( div_b_[0] <= offset_x ) || ( div_b_[1] <= offset_y ) || ( div_b_[2] <= offset_z ) ){
      std::cerr << "(In extractGRSDSignature325) offset values (" << offset_x << "," << offset_y << "," << offset_z << ") exceed voxel grid size (" << div_b_[0] << "," << div_b_[1] << "," << div_b_[2] << ")."<< std::endl;
      return Eigen::Vector3i::Zero();
    }
    subdiv_b_ = Eigen::Vector3i ( ceil( ( div_b_[0] - offset_x )*inverse_subdivision_size ), ceil( ( div_b_[1] - offset_y )*inverse_subdivision_size ), ceil( ( div_b_[2] - offset_z )*inverse_subdivision_size ) );
    subdivb_mul_ = Eigen::Vector3i ( 1, subdiv_b_[0], subdiv_b_[0] * subdiv_b_[1] );
    hist_num = subdiv_b_[0] * subdiv_b_[1] * subdiv_b_[2];
  }
  else if( subdivision_size < 0 ){
    std::cerr << "(In extractGRSDSignature325) Invalid subdivision size: " << subdivision_size << std::endl;
    return Eigen::Vector3i::Zero();
  }
  cloud_grsd.points.resize(hist_num);

  // Compute RSD
  pcl::RSDEstimation <T, T, pcl::PrincipalRadiiRSD> rsd;
  rsd.setInputCloud( cloud_downsampled_ptr );
  boost::shared_ptr< const pcl::PointCloud<T> > cloud_ptr = cloud.makeShared();
  rsd.setSearchSurface( cloud_ptr );
  rsd.setInputNormals( cloud_ptr );
#ifndef QUIET
  ROS_INFO("radius search: %f", std::max(rsd_radius_search, voxel_size/2 * sqrt(3)));
#endif
  rsd.setRadiusSearch(std::max(rsd_radius_search, voxel_size/2 * sqrt(3)));
  boost::shared_ptr< pcl::KdTree<T> > tree2 = boost::make_shared<pcl::KdTreeFLANN<T> > ();
  tree2->setInputCloud (cloud.makeShared());
  rsd.setSearchMethod(tree2);
  pcl::PointCloud<pcl::PrincipalRadiiRSD> radii;
  t1 = my_clock();
  rsd.compute(radii);
#ifndef QUIET
  ROS_INFO("RSD compute done in %f seconds.", my_clock()-t1);
#endif
  
  // Get rmin/rmax for adjacent 27 voxel
  t1 = my_clock();
  Eigen::MatrixXi relative_coordinates (3, 13);

  int idx = 0;
  
  // 0 - 8
  for( int i=-1; i<2; i++ )
  {
    for( int j=-1; j<2; j++ )
    {
      relative_coordinates( 0, idx ) = i;
      relative_coordinates( 1, idx ) = j;
      relative_coordinates( 2, idx ) = -1;
      idx++;
    }
  }
  // 9 - 11
  for( int i=-1; i<2; i++ )
  {
    relative_coordinates( 0, idx ) = i;
    relative_coordinates( 1, idx ) = -1;
    relative_coordinates( 2, idx ) = 0;
    idx++;
  }
  // 12
  relative_coordinates( 0, idx ) = -1;
  relative_coordinates( 1, idx ) = 0;
  relative_coordinates( 2, idx ) = 0;

  // Get transition matrix
  std::vector<int> types (radii.points.size());
 
  for (size_t idx = 0; idx < radii.points.size (); ++idx)
    types[idx] = getType(radii.points[idx].r_min, radii.points[idx].r_max);
  
  for (size_t idx = 0; idx < cloud_downsampled_ptr->points.size (); ++idx)
  {
    // calc hist_idx
    int hist_idx;
    if( hist_num == 1 ) hist_idx = 0;
    else{
      const int tmp_x = floor( cloud_downsampled_ptr->points[ idx ].x/voxel_size ) - min_b_[ 0 ] - offset_x;
      const int tmp_y = floor( cloud_downsampled_ptr->points[ idx ].y/voxel_size ) - min_b_[ 1 ] - offset_y;
      const int tmp_z = floor( cloud_downsampled_ptr->points[ idx ].z/voxel_size ) - min_b_[ 2 ] - offset_z;
      /* const int x_mul_y = div_b_[0] * div_b_[1]; */
      /* const int tmp_z = idx / x_mul_y - offset_z; */
      /* const int tmp_y = ( idx % x_mul_y ) / div_b_[0] - offset_y; */
      /* const int tmp_x = idx % div_b_[0] - offset_x; */
      if( ( tmp_x < 0 ) || ( tmp_y < 0 ) || ( tmp_z < 0 ) ) continue; // ignore idx smaller than offset.
      Eigen::Vector3i ijk = Eigen::Vector3i ( floor ( tmp_x * inverse_subdivision_size), floor ( tmp_y * inverse_subdivision_size), floor ( tmp_z * inverse_subdivision_size) );
      hist_idx = ijk.dot (subdivb_mul_);
    }

    int source_type = types[idx];
    std::vector<int> neighbors = grid.getNeighborCentroidIndices ( cloud_downsampled_ptr->points[idx], relative_coordinates);
    for (unsigned id_n = 0; id_n < neighbors.size(); id_n++)
    {
      int neighbor_type;
      if (neighbors[id_n] == -1)
        neighbor_type = EMPTY;
      else
        neighbor_type = types[neighbors[id_n]];

      // ignore EMPTY
      if( neighbor_type != EMPTY )
	cloud_grsd.points[ hist_idx ].histogram[ source_type + neighbor_type * 5 + id_n * 25 ]++;
    }
  }
#ifndef QUIET
  ROS_INFO("GRSD compute done in %f seconds.", my_clock()-t1);
#endif

  feature.resize( hist_num );
  if( is_normalize ){
    for( int h=0; h<hist_num; h++ ){
      feature[ h ].resize( GRSD_LARGE_DIM );
      for( int i=0; i<GRSD_LARGE_DIM; i++)
        feature[ h ][ i ] = cloud_grsd.points[ h ].histogram[ i ] * NORMALIZE_GRSD;
    }
  }
  else{
    for( int h=0; h<hist_num; h++ ){
      feature[ h ].resize( GRSD_LARGE_DIM );
      for( int i=0; i<GRSD_LARGE_DIM; i++)
        feature[ h ][ i ] = cloud_grsd.points[ h ].histogram[ i ];
    }
  }
  return subdiv_b_;
}

template <typename T>
void extractGRSDSignature325(pcl::VoxelGrid<T> grid, pcl::PointCloud<T> cloud, pcl::PointCloud<T> cloud_downsampled, std::vector<float> &feature, const float voxel_size, const bool is_normalize  ){
  std::vector< std::vector<float> > tmp( 1 );
  extractGRSDSignature325( grid, cloud, cloud_downsampled, tmp, voxel_size, 0, 0, 0, 0, is_normalize ); // for one signature
  feature = tmp[ 0 ];
}

//-----------------------------------
//* extract - PlusGRSD -
template <typename T>
Eigen::Vector3i extractPlusGRSDSignature110(pcl::VoxelGrid<T> grid, pcl::PointCloud<T> cloud, pcl::PointCloud<T> cloud_downsampled, std::vector< std::vector<float> > &feature, const float voxel_size, const int subdivision_size = 0, const int offset_x = 0, const int offset_y = 0, const int offset_z = 0, const bool is_normalize  ){
#ifndef QUIET
  ROS_INFO("rsd %f, normals %f, leaf %f", rsd_radius_search, normals_radius_search, voxel_size);
#endif
  feature.resize( 0 );
  boost::shared_ptr< const pcl::PointCloud<T> > cloud_downsampled_ptr;
  cloud_downsampled_ptr.reset (new pcl::PointCloud<T> (cloud_downsampled));
  
  //* for computing multiple GRSD with subdivisions
  int hist_num = 1;
  float inverse_subdivision_size = 0;
  Eigen::Vector3i div_b_= Eigen::Vector3i::Zero();
  Eigen::Vector3i min_b_= Eigen::Vector3i::Zero();
  Eigen::Vector3i subdiv_b_= Eigen::Vector3i::Zero();
  Eigen::Vector3i subdivb_mul_= Eigen::Vector3i::Zero();
  if( subdivision_size > 0 ){
    inverse_subdivision_size = 1.0 / subdivision_size;
    div_b_ = grid.getNrDivisions();
    min_b_ = grid.getMinBoxCoordinates();
    if( ( div_b_[0] <= offset_x ) || ( div_b_[1] <= offset_y ) || ( div_b_[2] <= offset_z ) ){
      std::cerr << "(In extractPlusGRSDSignature110) offset values (" << offset_x << "," << offset_y << "," << offset_z << ") exceed voxel grid size (" << div_b_[0] << "," << div_b_[1] << "," << div_b_[2] << ")."<< std::endl;
      return Eigen::Vector3i::Zero();
    }
    subdiv_b_ = Eigen::Vector3i ( ceil( ( div_b_[0] - offset_x )*inverse_subdivision_size ), ceil( ( div_b_[1] - offset_y )*inverse_subdivision_size ), ceil( ( div_b_[2] - offset_z )*inverse_subdivision_size ) );
    subdivb_mul_ = Eigen::Vector3i ( 1, subdiv_b_[0], subdiv_b_[0] * subdiv_b_[1] );
    hist_num = subdiv_b_[0] * subdiv_b_[1] * subdiv_b_[2];
  }
  else if( subdivision_size < 0 ){
    std::cerr << "(In extractPlusGRSDSignature110) Invalid subdivision size: " << subdivision_size << std::endl;
    return Eigen::Vector3i::Zero();
  }

  // Compute RSD
  pcl::RSDEstimation <T, T, pcl::PrincipalRadiiRSD> rsd;
  rsd.setInputCloud( cloud_downsampled_ptr );
  boost::shared_ptr< const pcl::PointCloud<T> > cloud_ptr = cloud.makeShared();
  rsd.setSearchSurface( cloud_ptr );
  rsd.setInputNormals( cloud_ptr );
#ifndef QUIET
  ROS_INFO("radius search: %f", std::max(rsd_radius_search, voxel_size/2 * sqrt(3)));
#endif
  rsd.setRadiusSearch(std::max(rsd_radius_search, voxel_size/2 * sqrt(3)));
  boost::shared_ptr< pcl::KdTree<T> > tree2 = boost::make_shared<pcl::KdTreeFLANN<T> > ();
  tree2->setInputCloud (cloud.makeShared());
  rsd.setSearchMethod(tree2);
  pcl::PointCloud<pcl::PrincipalRadiiRSD> radii;
  t1 = my_clock();
  rsd.compute(radii);
#ifndef QUIET
  ROS_INFO("RSD compute done in %f seconds.", my_clock()-t1);
  pcl::io::savePCDFile ("hoge.pcd", cloud_downsampled, false);
#endif
  
  // Get rmin/rmax for adjacent 27 voxel
  t1 = my_clock();
  Eigen::MatrixXi relative_coordinates (3, 13);

  int idx = 0;
  
  // 0 - 8
  for( int i=-1; i<2; i++ )
  {
    for( int j=-1; j<2; j++ )
    {
      relative_coordinates( 0, idx ) = i;
      relative_coordinates( 1, idx ) = j;
      relative_coordinates( 2, idx ) = -1;
      idx++;
    }
  }
  // 9 - 11
  for( int i=-1; i<2; i++ )
  {
    relative_coordinates( 0, idx ) = i;
    relative_coordinates( 1, idx ) = -1;
    relative_coordinates( 2, idx ) = 0;
    idx++;
  }
  // 12
  relative_coordinates( 0, idx ) = -1;
  relative_coordinates( 1, idx ) = 0;
  relative_coordinates( 2, idx ) = 0;

  Eigen::MatrixXi relative_coordinates_all (3, 26);
  relative_coordinates_all.block<3, 13>(0, 0) = relative_coordinates;
  relative_coordinates_all.block<3, 13>(0, 13) = -relative_coordinates;

  // Get transition matrix
  std::vector<int> types (radii.points.size());
 
  for (size_t idx = 0; idx < radii.points.size (); ++idx)
    types[idx] = getType(radii.points[idx].r_min, radii.points[idx].r_max);
  
  // TODO voxelization does not re-normalize the normals!
  for (size_t idx = 0; idx < cloud_downsampled_ptr->points.size (); ++idx)
    cloud_downsampled.points[idx].getNormalVector3fMap ().normalize ();

  // TODO use consts and iterators
  std::vector<Eigen::MatrixXi> transition_matrix_list (hist_num * NR_DIV, Eigen::MatrixXi::Zero (NR_CLASS, NR_CLASS));
  std::vector<Eigen::VectorXi> transitions_to_empty( hist_num, Eigen::VectorXi::Zero (NR_CLASS));

  for (size_t idx = 0; idx < cloud_downsampled_ptr->points.size (); ++idx)
  {
    // calc hist_idx
    int hist_idx;
    if( hist_num == 1 ) hist_idx = 0;
    else{
      const int tmp_x = floor( cloud_downsampled_ptr->points[ idx ].x/voxel_size ) - min_b_[ 0 ] - offset_x;
      const int tmp_y = floor( cloud_downsampled_ptr->points[ idx ].y/voxel_size ) - min_b_[ 1 ] - offset_y;
      const int tmp_z = floor( cloud_downsampled_ptr->points[ idx ].z/voxel_size ) - min_b_[ 2 ] - offset_z;
      /* const int x_mul_y = div_b_[0] * div_b_[1]; */
      /* const int tmp_z = idx / x_mul_y - offset_z; */
      /* const int tmp_y = ( idx % x_mul_y ) / div_b_[0] - offset_y; */
      /* const int tmp_x = idx % div_b_[0] - offset_x; */
      if( ( tmp_x < 0 ) || ( tmp_y < 0 ) || ( tmp_z < 0 ) ) continue; // ignore idx smaller than offset.
      Eigen::Vector3i ijk = Eigen::Vector3i ( floor ( tmp_x * inverse_subdivision_size), floor ( tmp_y * inverse_subdivision_size), floor ( tmp_z * inverse_subdivision_size) );
      hist_idx = ijk.dot (subdivb_mul_);
    }

    int source_type = types[idx];
    std::vector<int> neighbors = grid.getNeighborCentroidIndices ( cloud_downsampled_ptr->points[idx], relative_coordinates_all);
    Eigen::Vector3f source_normal = cloud_downsampled.points[idx].getNormalVector3fMap ();
    if( std::isfinite( cloud_downsampled.points[idx].normal_x ) && std::isfinite( cloud_downsampled.points[idx].normal_y ) && std::isfinite( cloud_downsampled.points[idx].normal_z ) ){
      for (unsigned id_n = 0; id_n < neighbors.size(); id_n++)
	{
	  int neighbor_type;
	  if (neighbors[id_n] == -1)
	    neighbor_type = EMPTY;
	  else
	    neighbor_type = types[neighbors[id_n]];
	  
	  // count transitions: TODO optimize Asako style :)  but what about empty?
	  if (neighbors[id_n] == -1)
	    transitions_to_empty[ hist_idx ](source_type)++;
	  else{
	    // compute angle between average normals of voxels
	    assert (source_type != EMPTY);
	    //        double sine = source_normal.cross (cloud_downsampled.points[neighbors[id_n]].getNormalVector3fMap ()).norm ();
	    //        int angle_bin = std::min (NR_DIV-1, (int) floor (sine * NR_DIV));
	    //        transition_matrix_list[angle_bin](source_type, neighbor_type)++;
	    //std::cout << cloud_downsampled.points[neighbors[id_n]].getNormalVector3fMap () << std::endl;
	    //std::cout << cloud_downsampled.points[neighbors[id_n]].normal_x << " " << cloud_downsampled.points[neighbors[id_n]].normal_y << " " << cloud_downsampled.points[neighbors[id_n]].normal_z << std::endl;
	    if( std::isfinite( cloud_downsampled.points[neighbors[id_n]].normal_x ) && std::isfinite( cloud_downsampled.points[neighbors[id_n]].normal_y ) && std::isfinite( cloud_downsampled.points[neighbors[id_n]].normal_z ) ) 
	      transition_matrix_list[ (std::min (NR_DIV-1, (int) floor (sqrt (source_normal.cross (cloud_downsampled.points[neighbors[id_n]].getNormalVector3fMap ()).norm ()) * NR_DIV)) )*hist_num + hist_idx ](source_type, neighbor_type)++;
	    else
	      transitions_to_empty[ hist_idx ](source_type)++;
	    //        double unsigned_cosine = fabs(source_normal.dot (cloud_downsampled.points[neighbors[id_n]].getNormalVector3fMap ()));
	    //        if (unsigned_cosine > 1) unsigned_cosine = 1;
	    //        int angle_bin = std::min (NR_DIV-1, (int) floor (acos (unsigned_cosine) / unit_angle));
	    //std::cerr << "angle difference = " << acos (unsigned_cosine) << "(" << unsigned_cosine << ") => index: " << angle_bin << std::endl;
	  }
	}
    }
  }

#ifndef QUIET
  std::cerr << "List of transition matrices by normal angle:" << std::endl;
  std::copy (transition_matrix_list.begin (), transition_matrix_list.end (), std::ostream_iterator<Eigen::MatrixXi> (std::cerr, "\n---\n"));  
  for( int h=0; h<hist_num; h++ )
    std::cerr << "Transitions to empty: " << transitions_to_empty[h].transpose () << std::endl;
#endif

  pcl::PointCloud<pcl::PlusGRSDSignature110> cloud_grsd;
  cloud_grsd.points.resize(hist_num);

  for( int h=0; h<hist_num; h++ ){  
    int nrf = 0;

    for ( int d=0; d<NR_DIV; d++ )
      {
	for (int i=0; i<NR_CLASS; i++)
	  for (int j=i; j<NR_CLASS; j++)
	    cloud_grsd.points[h].histogram[nrf++] = transition_matrix_list[ d * hist_num + h ](i, j);
      }
    for (int it = 0; it < NR_CLASS; ++it)
      cloud_grsd.points[h].histogram[nrf++] = transitions_to_empty[ h ][it];
    //std::cerr << std::endl;

#ifndef QUIET
    std::cerr << "PlusGRSDS: " << cloud_grsd.points[0] << std::endl;
#endif
  }

#ifndef QUIET
  ROS_INFO("GRSD compute done in %f seconds.", my_clock()-t1);
#endif

  feature.resize( hist_num );
  if( is_normalize ){
    for( int h=0; h<hist_num; h++ ){
      feature[ h ].resize( 110 );
      for( int i=0; i<110; i++)
        feature[ h ][ i ] = cloud_grsd.points[ h ].histogram[ i ] * NORMALIZE_GRSD;
    }
  }
  else{
    for( int h=0; h<hist_num; h++ ){
      feature[ h ].resize( 110 );
      for( int i=0; i<110; i++)
        feature[ h ][ i ] = cloud_grsd.points[ h ].histogram[ i ];
    }
  }
  return subdiv_b_;
}

template <typename T>
void extractPlusGRSDSignature110(pcl::VoxelGrid<T> grid, pcl::PointCloud<T> cloud, pcl::PointCloud<T> cloud_downsampled, std::vector<float> &feature, const float voxel_size, const bool is_normalize  ){
  std::vector< std::vector<float> > tmp( 1 );
  extractPlusGRSDSignature110( grid, cloud, cloud_downsampled, tmp, voxel_size, 0, 0, 0, 0, is_normalize ); // for one signature
  feature = tmp[ 0 ];
}

//--------------
//* concatenate
const std::vector<float> concVector( const std::vector<float> v1, const std::vector<float> v2 ){
  std::vector<float> vec = v1;
  vec.insert(vec.end(), v2.begin(), v2.end());
  return vec;
}

//--------------
//* VOSCH
template <typename PointT>
Eigen::Vector3i extractVOSCH(pcl::VoxelGrid<PointT> grid, pcl::PointCloud<PointT> cloud, pcl::PointCloud<PointT> cloud_downsampled, std::vector< std::vector<float> > &feature, int color_threshold_r, int color_threshold_g, int color_threshold_b, const float voxel_size, const int subdivision_size = 0, const int offset_x = 0, const int offset_y = 0, const int offset_z = 0, const bool is_normalize  ){
  std::vector< std::vector<float> > grsd;
  Eigen::Vector3i subdiv_b_ = extractGRSDSignature21( grid, cloud, cloud_downsampled, grsd, voxel_size, subdivision_size, offset_x, offset_y, offset_z, is_normalize );
  std::vector< std::vector<float> > c3_hlac;
  extractC3HLACSignature117( grid, cloud_downsampled, c3_hlac, color_threshold_r, color_threshold_g, color_threshold_b, voxel_size, subdivision_size, offset_x, offset_y, offset_z );

  const int hist_num = grsd.size();
  for( int h=0; h<hist_num; h++ )
    feature.push_back ( concVector( grsd[ h ], c3_hlac[ h ] ) );
  return subdiv_b_;
}

template <typename PointT>
void extractVOSCH(pcl::VoxelGrid<PointT> grid, pcl::PointCloud<PointT> cloud, pcl::PointCloud<PointT> cloud_downsampled, std::vector<float> &feature, int color_threshold_r, int color_threshold_g, int color_threshold_b, const float voxel_size, const int subdivision_size = 0, const int offset_x = 0, const int offset_y = 0, const int offset_z = 0 ){
  std::vector< std::vector<float> > tmp;
  extractVOSCH( grid, cloud, cloud_downsampled, tmp, color_threshold_r, color_threshold_g, color_threshold_b, voxel_size ); // for one signature
  feature = tmp[ 0 ];
}
