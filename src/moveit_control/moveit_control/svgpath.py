import svgpathtools
import numpy as np

def svg_to_waypoints(svg_file, scale_factor=0.001, z_offset=0.1, resolution=0.01):
    """
    Convertit un fichier SVG en une liste de waypoints 3D.
    :param svg_file: Chemin vers le fichier SVG.
    :param scale_factor: Facteur de mise à l'échelle du SVG aux unités du robot (mètres).
    :param z_offset: Hauteur Z des points de la trajectoire.
    :param resolution: Distance approximative entre les points discrets sur le chemin.
    :return: Liste de numpy arrays [x, y, z] représentant les waypoints.
    """
    try:
        paths, attributes = svgpathtools.svg2paths(svg_file)
    except Exception as e:
        print(f"Erreur lors du chargement du fichier SVG: {e}")
        return []

    waypoints = []
    for path in paths:
        # Discrétiser le chemin
        # path.length() donne la longueur du chemin
        # resolution est la distance entre les points. Plus la resolution est petite, plus il y a de points.
        num_segments = int(path.length() / resolution)
        if num_segments == 0:
            num_segments = 1 # Assure au moins un point pour des chemins très courts

        for i in range(num_segments + 1):
            t = i / num_segments
            point = path.point(t) # Obtient un point complexe (x + yi)
            x = point.real * scale_factor
            y = point.imag * scale_factor
            z = z_offset
            waypoints.append(np.array([x, y, z]))
    return waypoints

# Exemple d'utilisation
# svg_path = "votre_dessin.svg"
# waypoints = svg_to_waypoints(svg_path, scale_factor=0.001, z_offset=0.1, resolution=0.005)
# print(f"Nombre de waypoints générés: {len(waypoints)}")