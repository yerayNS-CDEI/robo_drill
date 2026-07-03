def generate_sdf_code(num_filas):
    # Primera parte del código
    sdf_code = """
<sdf version='1.7'>
  <world name='agropolis'>
    <light name='sun' type='directional'>
      <cast_shadows>1</cast_shadows>
      <pose>0 0 10 0 -0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.03 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
      <spot>
        <inner_angle>0</inner_angle>
        <outer_angle>0</outer_angle>
        <falloff>0</falloff>
      </spot>
    </light>
    <model name='ground_plane'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>100</mu>
                <mu2>50</mu2>
              </ode>
              <torsional>
                <ode/>
              </torsional>
            </friction>
            <contact>
              <ode/>
            </contact>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type='adiabatic'/>
    <physics type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>1</shadows>
    </scene>
    <audio>
      <device>default</device>
    </audio>
    <wind/>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>41.288011</latitude_deg>
      <longitude_deg>2.045465</longitude_deg>
      <elevation>0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>
"""

    # Segunda parte del código
    base_x_offset = 2.9
    base_fila_code = """
    <model name='fila{fila}'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <mesh>
              <uri>file://models/pasillo/meshes/pasillo.dae</uri>
            </mesh>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <mesh>
              <uri>file://models/pasillo/meshes/pasillo.dae</uri>
            </mesh>
          </geometry>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <pose>{x_offset} -0.72112 0 0 -0 0</pose>
    </model>
"""
    base_model_code = """
    <model name='vinya_{index}_fila_{fila}'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <mesh>
              <uri>file://models/vinya/meshes/vinya.dae</uri>
            </mesh>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <mesh>
              <uri>file://models/vinya/meshes/vinya.dae</uri>
            </mesh>
          </geometry>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <pose>{x_offset} {y} 0.03 0 -0 1.57</pose>
    </model>
"""

    y_positions = [-5.9, -4.40133, -2.94517, -1.60177, -7.64563, -9.06325, -10.4717, -11.8519,
                   -13.6368, -14.9995, -16.381, -17.8271, -19.6679, -21.121, -22.6011, -24.1275,
                   0.038395, 1.51569, 2.8899, 4.31805, 6.09459, 7.52059, 8.93263, 10.3418,
                   12.1, 13.6, 15, 16.404, 18.2453, 19.8052, 21.2672, 22.8]

    for fila in range(1, num_filas + 1):
        x_offset = base_x_offset * (fila - 1)
        sdf_code += base_fila_code.format(fila=fila, x_offset=x_offset)
        for index, y in enumerate(y_positions, start=1):
            sdf_code += base_model_code.format(index=index, fila=fila, x_offset=x_offset, y=y)

    # Tercera parte del código
    sdf_code += """
    <state world_name='default'>
      <sim_time>372 204000000</sim_time>
      <real_time>393 945473628</real_time>
      <wall_time>1740217615 737199426</wall_time>
      <iterations>372204</iterations>
      <model name='ground_plane'>
        <pose>0 0 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>0 0 0 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
"""

    # Cuarta parte del código
    for fila in range(1, num_filas + 1):
        x_offset = base_x_offset * (fila - 1)
        sdf_code += f"""
<model name='fila{fila}'>
  <pose>{x_offset} -0.72112 0 0 -0 0</pose>
  <scale>1 1 1</scale>
  <link name='link'>
    <pose>{x_offset} -0.72112 0 0 -0 0</pose>
  </link>
</model>
"""
        for index, y in enumerate(y_positions, start=1):
            sdf_code += f"""
<model name='vinya_{index}_fila_{fila}'>
  <pose>{x_offset} {y} 0.03 0 -0 1.57</pose>
  <scale>1 1 1</scale>
  <link name='link'>
    <pose>{x_offset} {y} 0.03 0 -0 1.57</pose>
  </link>
</model>
"""

    # Última parte del código
    sdf_code += """
      <light name='sun'>
        <pose>0 0 10 0 -0 0</pose>
      </light>
    </state>
    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose>10.5139 -7.1422 5.32307 0 0.267643 2.9162</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>
  </world>
</sdf>
"""
    return sdf_code

# Example usage
num_filas = 10  # Specify the number of rows you want
sdf_code = generate_sdf_code(num_filas)

# Save the generated SDF code to a file
with open("agropolis.world", "w") as file:
    file.write(sdf_code)

print("SDF world file generated: agropolis.world")



