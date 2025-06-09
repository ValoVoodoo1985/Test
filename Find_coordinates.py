import geopy.distance
import numpy as np

def angle_between_2d_vectors_360(vec1, vec2):
    if vec1.shape != (2,) or vec2.shape != (2,):
        raise ValueError("This function is designed for 2D vectors only (shape (2,)).")

    norm_vec1 = np.linalg.norm(vec1)
    norm_vec2 = np.linalg.norm(vec2)

    if norm_vec1 == 0 or norm_vec2 == 0:
        return 0.0

    dot_product = np.dot(vec1, vec2)

    det_val = vec1[0] * vec2[1] - vec1[1] * vec2[0]

    angle_rad = np.arctan2(det_val, dot_product)

    angle_deg = np.degrees(angle_rad)

    if angle_deg < 0:
        angle_deg += 360

    return angle_deg

azimuth = 335
azimuth_offset = 360 - azimuth
meters_per_pixel = 0.38
control_point_geodesic = (50.603694, 30.650625)
image_center_point = np.array([320, 256])
image_control_point = np.array([558, 328])

result_vector = image_center_point - image_control_point # Цей вектор потрібен щоб знайти кут між північю і цим вектором в просторі зображення.

image_north = np.array([0, -1]) # Північ перевернута, томущо різниця векторів розрахована в системі коорденат зображення.

image_bearing = angle_between_2d_vectors_360(image_north, result_vector) # Розраховую напрямок від контрольної точку до центру зображення.

image_space_bearing_in_world_space_azimuth = image_bearing - azimuth_offset # Переводжу напрямок в світовий простір.

result_vector_lenth = np.linalg.norm(result_vector) # Дистанція між контрольною точкою і центром зображення в пікселях.

distance_in_meters = result_vector_lenth * meters_per_pixel # Дистанція в метрах.

image_center_geodesic_coordinates = geopy.distance.distance(meters = distance_in_meters).destination(control_point_geodesic, bearing = image_space_bearing_in_world_space_azimuth) # Знаючи напрямок, дистанцію і координати контрольної точки, знаходжу координати центру зображення.

print('Distance: ', distance_in_meters)
print('Bearing: ', image_space_bearing_in_world_space_azimuth)
print('Geodesic coordinates: ', image_center_geodesic_coordinates[0], image_center_geodesic_coordinates[1])


