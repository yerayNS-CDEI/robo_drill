# Colocación de las viñas a una distancia específica entre palos
def generate_sdf_code(num_filas, palos_por_fila, vinyas_por_fila):
    sdf_code = """<?xml version="1.0" ?>
<sdf version='1.7'>
  <world name='default'>
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
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </visual>
      </link>
    </model>
"""

    distancia_entre_filas = 4.0  # Distancia entre filas
    distancias_vinyas = [0.9, 1.56, 1.4, 1.575]  # Distancias entre viñas
    distancia_palo_siguiente = 0.725  # Distancia entre la última viña y el siguiente palo
    distancia_total_entre_palos = sum(distancias_vinyas) + distancia_palo_siguiente  # 6.16

    for fila in range(num_filas):
        y_offset = fila * distancia_entre_filas  # Posición en el eje Y

        x_offset = 0  # Posición en el eje X para la primera vinya de la fila
        for palo_idx in range(palos_por_fila[fila]):
            # Agregar el palo
            sdf_code += f"""
    <model name='unit_cylinder_fila_{fila+1}_palo_{palo_idx+1}'>
      <static>true</static>
      <pose>{x_offset} {y_offset} 1.5 0 0 0</pose>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <cylinder>
              <radius>0.075</radius>
              <length>3</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name='visual'>
          <geometry>
            <cylinder>
              <radius>0.075</radius>
              <length>3</length>
            </cylinder>
          </geometry>
        </visual>
      </link>
    </model>
"""

            # **Evitar colocar viñas después del último palo**
            if palo_idx < palos_por_fila[fila] - 1:  
                # Agregar las 4 viñas entre cada par de palos
                for vinya_idx in range(4):
                    x_offset += distancias_vinyas[vinya_idx]  # Avanzar en X según la distancia definida
                    sdf_code += f"""
    <model name='vinya_{vinya_idx+1}_palo_{palo_idx+1}_fila_{fila+1}'>
      <static>true</static>
      <pose>{x_offset} {y_offset} 0 0 0 0</pose>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <mesh>
              <uri>file://models/vinya/meshes/vinya.dae</uri>
            </mesh>
          </geometry>
        </collision>
        <visual name='visual'>
          <geometry>
            <mesh>
              <uri>file://models/vinya/meshes/vinya.dae</uri>
            </mesh>
          </geometry>
        </visual>
      </link>
    </model>
"""

                # Después de colocar las 4 viñas, sumamos la distancia extra para el siguiente palo
                x_offset += distancia_palo_siguiente

    # Cerrar el archivo SDF correctamente
    sdf_code += """
  </world>
</sdf>
"""

    return sdf_code


# Parámetros del viñedo
num_filas = 7
palos_por_fila = [11] * num_filas  # 11 palos por fila
vinyas_por_fila = [40] * num_filas  # 40 viñas por fila

sdf_code = generate_sdf_code(num_filas, palos_por_fila, vinyas_por_fila)

# Guardar el archivo
with open("castelldefels_vineyard.world", "w") as file:
    file.write(sdf_code)

print("SDF world file generated: castelldefels_vineyard.world")

