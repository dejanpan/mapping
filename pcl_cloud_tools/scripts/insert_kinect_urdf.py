import os, sys

stext = '<joint name="double_stereo_frame_joint" type="fixed">'

rtext = '<joint name="openni_rgb_frame_joint" type="fixed">\n\
    <origin rpy="0.0074253744 0.0418016634 -0.0065419807" xyz="0.0440562178 -0.0135760086 0.1129906398"/>\n\
    <parent link="head_plate_frame"/>\n\
    <child link="openni_rgb_frame"/>\n\
  </joint>\n\
  <link name="openni_rgb_frame">\n\
    <inertial>\n\
      <mass value="0.01"/>\n\
      <origin xyz="0 0 0"/>\n\
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>\n\
    </inertial>\n\
    <visual>\n\
      <origin rpy="0 0 0" xyz="0 0 0"/>\n\
      <geometry>\n\
	<box size="0.01 0.01 0.01"/>\n\
      </geometry>\n\
    </visual>\n\
  </link>\n\
  <joint name="openni_rgb_optical_frame_joint" type="fixed">\n\
    <origin rpy="-1.5707963268 -0.0000000000 -1.5707963268" xyz="0.0000000000 0.0000000000 0.0000000000"/>\n\
    <parent link="openni_rgb_frame"/>\n\
    <child link="openni_rgb_optical_frame"/>\n\
  </joint>\n\
  <link name="openni_rgb_optical_frame"/>\n\
  <joint name="double_stereo_frame_joint" type="fixed">'

def serchReplace(path):
    input = open(path)
    print "urdf:", path
    out = path+'.tmp'
    output = open(out, 'w')
    for s in input.xreadlines():
        output.write(s.replace(stext, rtext))
    input.close()
    output.close()
    os.rename(out, path)

def usage():
     print "\nUsage: python ", sys.argv[0], "<robot_uncalibrated_x.x.x.xml> \n"

if __name__ == "__main__":
    if sys.argv.__len__() == 1:
        usage()
        sys.exit(2)
    path = sys.argv[1]
    serchReplace(path)
