import numpy as np 


def scan_conversion(box_points, rect_coords):

	# seperating the coordinates into 1,2,3 and 4 points
	# the points are in clock-wise direction 1->2->3->4->1
	b1 = box_points[0]
	b2 = box_points[1]
	b3 = box_points[2]
	b4 = box_points[3]

	# forming two triangles from rectangle 1->2->4 and 2->3->4
	tri1 = np.array([b1,b2,b4])
	tri2 = np.array([b2,b3,b4])
	collect_ind = []
	# for each point check if the point is in traingle 1 or teiangle 2
	for ind,pt in enumerate(rect_coords):
		ans1 = in_triangle(tri1, np.array(pt[:2]))
		ans2 = in_triangle(tri2, np.array(pt[:2]))
		if (ans1 or ans2):
			collect_ind.append(ind)
	# retain the points which are present in one of the triangles
	fin_points = rect_coords[collect_ind]
	return fin_points


''' Ref: https://stackoverflow.com/questions/5922027/how-to-determine-if-a-point-is-within-a-quadrilateral '''

def in_triangle(vertices, P):
	A = vertices[0]
	B = vertices[1]
	C = vertices[2]
	v0 = C - A
	v1 = B - A
	v2 = P - A

	# Compute dot products
	dot00 = np.dot(v0, v0)
	dot01 = np.dot(v0, v1)
	dot02 = np.dot(v0, v2)
	dot11 = np.dot(v1, v1)
	dot12 = np.dot(v1, v2)

	# Compute barycentric coordinates
	invDenom = 1 / (dot00 * dot11 - dot01 * dot01)
	u = (dot11 * dot02 - dot01 * dot12) * invDenom
	v = (dot00 * dot12 - dot01 * dot02) * invDenom

	# Check if point is in triangle
	return ((u >= 0) and (v >= 0) and (u + v <= 1))




