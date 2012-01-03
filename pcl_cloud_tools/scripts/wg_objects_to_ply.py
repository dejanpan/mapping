import os, sys

def fillPreamble (num_ver, num_faces):
    preamble = "ply\nformat ascii 1.0\nelement vertex " + str(num_ver) + "\n" + \
    "property float x\nproperty float y\nproperty float z\nelement face " + str(num_faces) + "\n" + \
    "property list uchar int vertex_indices\nend_header\n"
    return preamble

def convertToPly(in_file, out_file):
    print "in_file:", in_file
    input = open(in_file)
    output = open(out_file, 'w')
    #write preamble
    
    vertex_line = ['', '', '']
    line_count = 0;
    vertices_string_tmp = ''
    for s in input.xreadlines():
        #find vertices
        vertex_x = s.find("x: ")
        if vertex_x >= 0:
#            print s.split("x: ")[1]
            vertex_line[0] = s.split("x: ")[1].rstrip('\n')
        vertex_y = s.find("y: ")
        if vertex_y >= 0:
#                print s.split("y: ")[1]
                vertex_line[1] = s.split("y: ")[1].rstrip('\n')
        vertex_z = s.find("z: ")
        if vertex_z >= 0:
#                print s.split("z: ")[1]
                vertex_line[2] = s.split("z: ")[1].rstrip('\n')
        #write the line
        if vertex_line[0] != '' and vertex_line[1] != '' and vertex_line[2] != '':
#            print "vertex_line: ", line_count, ' '.join(vertex_line)
            #output.write(' '.join(vertex_line) + '\n')
            #output.write('\n')
            vertices_string_tmp = vertices_string_tmp + ' '.join(vertex_line) + '\n'
            line_count = line_count + 1
            vertex_line = ['', '', '']            

        #find faces
        faces = s.find("triangles: [")
        if faces >= 0:
            faces_string = s.split("triangles: [")[1].rstrip("\n").rstrip("]")
            faces_list = faces_string.split(', ')
            #print vertices_list

    print "#vertices: ", line_count
    
# 0 shall be replaced with vertices_list.__len__()/3 once I figure it out how faces are organized
    output.write(fillPreamble(line_count, 0))
    output.write(vertices_string_tmp)
            
    for (counter, v) in enumerate(faces_list):
        if counter%3 == 0:
            output.write('3 ' + faces_list[counter] + ' ' + faces_list[counter + 1] + ' ' + faces_list[counter + 2] + '\n')
    print "#faces: ", faces_list.__len__()/3
    input.close()
    output.close()

if __name__ == "__main__":
    in_file = sys.argv[1]
    out_file = sys.argv[2]
    convertToPly (in_file, out_file)
