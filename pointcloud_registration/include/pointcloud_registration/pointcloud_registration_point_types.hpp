namespace pcl
{

struct PointXYZINormal
{
  float x;
  float y;
  float z;
  float intensity;
  float normal[3];
  float curvature;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN_128;
inline std::ostream& operator<<(std::ostream& os, const PointXYZINormal& p)
{
  os << "("<<p.x<<","<<p.y<<","<<p.z<<" - "<<p.intensity<<" - "<<p.normal[0]<<","<<p.normal[1]<<","<<p.normal[2]<<" - "<<p.curvature<<")";
  return os;
}

struct PointXYZINormalScanIndex
{
  float x;
  float y;
  float z;
  float intensity;
  float normal[3];
  float curvature;
  int scan_index;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN_128;
inline std::ostream& operator<<(std::ostream& os, const PointXYZINormalScanIndex& p)
{
  os << "("<<p.x<<","<<p.y<<","<<p.z<<" - "<<p.intensity<<" - "<<p.normal[0]<<","<<p.normal[1]<<","<<p.normal[2]<<" - "<<p.curvature<<" - "<<p.scan_index<<")";
  return os;
}

struct SpinImageLocal
{
  uint32_t histogram[100];
};
inline std::ostream& operator<<(std::ostream& os, const SpinImageLocal& p)
{
  for (int i=0; i<100; ++i) os<<(i==0?"(":"")<<p.histogram[i]<<(i<99 ? ", " : ")");
  return os;
}

}  // End namespace
