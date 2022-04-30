import numpy as np

def check_colinearity(p1,p2,p3):
    distance_p1_p2 = np.linalg.norm(p1-p2)
    distance_p1_p3 = np.linalg.norm(p1-p3)
    distance_p2_p3 = np.linalg.norm(p2-p3)
    distances = [distance_p1_p2,distance_p1_p3,distance_p2_p3]
    largest_distance = max(distances)
    distances.remove(largest_distance)
    return True if (f'{largest_distance:.2f}' == f'{sum(distances):.2f}') else False

#Recebe uma lista de pontos, sendo os pontos numpyarrays
def simplify_path(path):
    simplified_path = []
    simplified_path.append(path[0])
    for i in range(2,len(path)-1):
        if not check_colinearity(path[i-2], path[i-1], path[i]):
            simplified_path.append(path[i-1])
    simplified_path.append(path[len(path)-1])
    return simplified_path