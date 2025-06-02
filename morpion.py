from pyniryo import *
from minimax import *
import copy
import warnings

warnings.filterwarnings("ignore", category=DeprecationWarning)

# enregistre la position relative en x et y des cases dans le repère dynamique Morpion en partant de haut à gauche et en finissant en bas à droite
caseMorpion = [
    [[0.025, 0.155], [0.09, 0.155], [0.155, 0.155]],
    [[0.025, 0.09],  [0.09, 0.09],  [0.155, 0.09]],
    [[0.025, 0.025], [0.09, 0.025], [0.155, 0.025]]
    ]

homeJoints = [0, 0.5, -1.25, 0, 0, 0]
observationJoints = [0, 0.22, -0.4, 0, -1.6, 0] 

pickPose = PoseObject(0.2768, -0.1265, 0.1091, 0.177, 1.251, -1.431)

def detect_objects_positions(robot, coordCases, game, depth = 0.32):
    img_compressed = robot.get_img_compressed()
    img = uncompress_image(img_compressed)
    img_threshold = threshold_hsv(img, *ColorHSV.ANY.value)
    # appliquer des transformations morphologiques (par exemple, ouverture) pour améliorer l'image
    img_threshold = morphological_transformations(img_threshold, morpho_type=MorphoType.OPEN,
                                                  kernel_shape=(11, 11), kernel_type=KernelType.ELLIPSE)
    # trouver les contours dans l'image seuillée
    cnts = biggest_contours_finder(img_threshold, 10)  # Trouver les 10 plus gros contours
    
    x_origin = 195
    y_origin = 305

    for cnt in cnts:
        # trouver le barycentre de chaque contour
        cnt_barycenter = get_contour_barycenter(cnt)
        cx, cy = cnt_barycenter
        if 180 < cx < 420 and 70 < cy < 300 :
            # afficher la position du barycentre (x, y) de l'objet
            # print(f"Objet détecté à la position (x: {cx - x_origin}, y: {y_origin - cy})")

            # obtenir les paramètres de la caméra
            mtx, dist = robot.get_camera_intrinsics()

            # paramètres de calibration de la caméra
            fx = mtx[0, 0]
            fy = mtx[1, 1]

            # conversion pixels → mètres avec les focales
            x_real = (cx - x_origin) * depth / fx
            y_real = (y_origin - cy) * depth / fy
            
            i, j = get_closest_case([x_real, y_real], coordCases)
            shape = detect_shape(cnt)
            game[i][j] = shape

            # pour visualisation, dessiner le contour et le barycentre
            img_debug = draw_contours(img_threshold, [cnt])
            img_debug = draw_barycenter(img_debug, cx, cy)

            # afficher l'image de débogage
            # show_img_and_wait_close("Image avec barycentre", img_debug)

    return game
    

def move_to_object(robot, object_position, z_position=0.1, depth=0.32):
    """
    Déplace le robot à la position de l'objet détecté à partir d'une image.
    
    :param robot: instance de NiryoRobot
    :param object_position: tuple (x, y) de la position en pixels dans l'image
    :param z_position: hauteur cible pour le robot
    :param depth: estimation de la profondeur (mètres)
    """
    # obtenir les paramètres de la caméra
    mtx, dist = robot.get_camera_intrinsics()

    # paramètres de calibration de la caméra
    fx = mtx[0, 0]
    fy = mtx[1, 1]

    # conversion pixels → mètres avec les focales
    x_real = object_position[0] * depth / fx
    y_real = object_position[1] * depth / fy
    z_real = z_position

    print(f"Objet détecté à {object_position} (px)")
    print(f"Coordonnées réelles estimées : x={x_real:.3f} m, y={y_real:.3f} m, z={z_real:.3f} m")

    # créer une pose robot et se déplacer
    pose = PoseObject(x=x_real, y=y_real, z=z_real, roll=0.0, pitch=1.57, yaw=0.0)
    robot.move_pose(pose, "Morpion")
    
def get_closest_case(position, cases):
    min_distance = float('inf')
    closest_coords = (0, 0)
    for i in range(3):
        for j in range(3):
            case = cases[i][j]
            dx = position[0] - case[0]
            dy = position[1] - case[1]
            distance = (dx**2 + dy**2)**0.5
            if distance < min_distance:
                min_distance = distance
                closest_coords = (i, j)
    return closest_coords
    
def detect_shape(cnt):
    """
    Détecte la forme (cercle ou carré) d'un contour.
    :param cnt: contour
    :return: int - 1 si 'cercle' ou 2 si 'carré'
    """
    approx = cv2.approxPolyDP(cnt, 0.04 * cv2.arcLength(cnt, True), True)
    if len(approx) == 4:
        return 2
    else:
        return 1
        
def check_winner(game):

    joueurs = {1: "joueur", 2: "robot"}

    # vérifie lignes et colonnes
    for i in range(3):
        if game[i][0] == game[i][1] == game[i][2] != 0:
            print(f"Le {joueurs[game[i][0]]} a gagné (ligne {i+1}).")
            return False
        if game[0][i] == game[1][i] == game[2][i] != 0:
            print(f"Le {joueurs[game[0][i]]} a gagné (colonne {i+1}).")
            return False
    
    # vérifie les diagonales
    if game[0][0] == game[1][1] == game[2][2] != 0:
        print(f"Le {joueurs[game[0][0]]} a gagné (diagonale principale).")
        return False
    if game[0][2] == game[1][1] == game[2][0] != 0:
        print(f"Le {joueurs[game[0][2]]} a gagné (diagonale secondaire).")
        return False

    # vérifie égalité
    if all(cell != 0 for row in game for cell in row):
        print("La partie est terminée par égalité.")
        return False

    print("La partie continue.")
    return True

def checkWhoStart(robot):
    img_compressed = robot.get_img_compressed()
    img = uncompress_image(img_compressed)
    img_threshold = threshold_hsv(img, *ColorHSV.ANY.value)
    # Appliquer des transformations morphologiques (par exemple, ouverture) pour améliorer l'image
    img_threshold = morphological_transformations(img_threshold, morpho_type=MorphoType.OPEN,
                                                  kernel_shape=(11, 11), kernel_type=KernelType.ELLIPSE)
    
    cnts = biggest_contours_finder(img_threshold, 5)

    for cnt in cnts:
        # Trouver le barycentre de chaque contour
        cnt_barycenter = get_contour_barycenter(cnt)
        cx, cy = cnt_barycenter
        if 180 < cx < 420 and 70 < cy < 300 :
            shape = detect_shape(cnt)
            return shape
    return 0

    


def play(game, coup):
    i, j = coup
    robot.move_pose(pickPose)
    robot.close_gripper(400)

    robot.move_joints(observationJoints)
    robot.move_pose(PoseObject(caseMorpion[i][j][0], caseMorpion[i][j][1], 0.09, 0, 1.57, 0), "Morpion")
    robot.open_gripper(400)
    game[i][j] = 2

    robot.move_joints(observationJoints)
    
if __name__ == '__main__':
    robot = NiryoRobot("10.10.10.10")
    robot.calibrate_auto()
    
    robot.set_brightness(1.3) # paramètres à changer en fonction de la luminosité de la pièce
    robot.set_contrast(1.0)
    robot.set_saturation(1.1)
    
    origin2 = [0.2029, 0.0868, 0.0188]
    x_axe2 = [0.2004, -0.0858, 0.0190]
    y_axe2 = [0.3718, 0.0836,0.0272]
    robot.save_dynamic_frame_from_points("Morpion", "Un repère dynamique prenant comme origine le repère en bas en gauche du workspace", origin2, x_axe2, y_axe2)
    
    # état du jeu (0 si pas de pion, 2 pour un carré, 1 pour un cercle)
    game = [[0, 0, 0],
            [0, 0, 0],
            [0, 0, 0]
            ]
    newGame = copy.deepcopy(game)

    robot.move_joints(observationJoints)
    robot.open_gripper(400)
    
    playing = True

    print("La partie peut commencer !")
    start = 0
    while start == 0:
        start = int(input("Rentrez 1 pour commencer, 2 pour que le robot commence !"))
        assert start == 1 or start == 2, "Ne rentrez que 1 ou 2"
        robot.wait(3)
    change = False
    if start == 2:
        change = True
        print("Je commence")
    else:
        print("Commencez !")
    robot.wait(5)
    while playing:
        while not change:
            print("En attente ...")
            robot.wait(3)
            detect_objects_positions(robot, caseMorpion, newGame)

            # conserver les cases déjà remplies
            for i in range(3):
                for j in range(3):
                    if game[i][j] != 0:
                        newGame[i][j] = game[i][j]

            # vérifier si un nouveau pion a été posé
            joueur_a_joue = False
            for i in range(3):
                for j in range(3):
                    if game[i][j] == 0 and newGame[i][j] != 0:
                        joueur_a_joue = True

            if joueur_a_joue:
                game = copy.deepcopy(newGame)
                change = True

        print("Etat du jeu :", newGame)
        robot.led_ring_chase([15, 255, 50])

        playing = check_winner(game)
        if not playing:
            break

        coup = meilleur_coup(game)
        play(game, coup)

        # mettre à jour les deux matrices après le coup joué par le robot
        newGame = copy.deepcopy(game)
        change = False
        playing = check_winner(game)
    
    robot.move_joints(homeJoints)
    robot.close_connection()
