import xacro

def make_urdf():
    """
    based on https://github.com/open-dynamic-robot-initiative/robot_properties_solo/blob/master/setup.py

    alternatively this can done by using the xacro command line tool:
    rosrun xacro xacro brunhilde.urdf.xacro > brunhilde.urdf

    Validation can also be done in ros:
    check_urdf brunhilde.urdf

    """
    xacro_file = './brunhilde.urdf.xacro'
    urdf_file = 'brunhilde.urdf'

    doc = xacro.process_file(xacro_file)
    out = xacro.open_output(urdf_file)

    encoding = {}
    out.write(doc.toprettyxml(indent="  ", **encoding))
    out.close()



if __name__ == '__main__':
    make_urdf()