uint32 size	# frontier size (amount of cells)

# Points
geometry_msgs/Point    center_point		# Frontier center point
geometry_msgs/Point[]  cells_points 	# vector containing points of all frontier cells

# Cells indexes of map array
uint32      center_cell 	    # cell corresponding to center_point
uint32[] 	cells        	    # vector containing all frontier cells
