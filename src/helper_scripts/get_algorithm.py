import sys
sys.path.append("./Navigation/")

from Navigation.algorithms.CenterRowAlgorithm import CenterRowAlgorithm
from Navigation.algorithms.CheckRowEnd import CheckRowEnd
from Navigation.algorithms.HoughAlgorithm import HoughAlgorithm
from Navigation.algorithms.MiniContoursAlgorithm import MiniContoursAlgorithm
from Navigation.algorithms.MiniContoursDownwards import MiniContoursDownwards
from Navigation.algorithms.ScanningAlgorithm import ScanningAlgorithm
from Navigation.algorithms.SeesawAlgorithm import SeesawAlgorithm
from Navigation.algorithms.Algorithm import Algorithm

import os.path as path
from omegaconf import OmegaConf


algo_list = [('hough', HoughAlgorithm), ('center_row', CenterRowAlgorithm), ('mini_contour', MiniContoursAlgorithm),
             ('mini_contour_downward', MiniContoursDownwards), ('scanning', ScanningAlgorithm), ('check_row_end', CheckRowEnd),
             ('seesaw', SeesawAlgorithm)]


def get_algorithm(algorithm_name) -> Algorithm:
    # verify that config file for algorithm exists
    if not path.isfile(f'./Navigation/config/algorithm/{algorithm_name}.yaml'):
        print(f"--alg config for {algorithm_name} is not defined in ./config/algorithm/")
        sys.exit()

    # set config
    config = OmegaConf.load(f'./Navigation/config/algorithm/{algorithm_name}.yaml')

    # return algorithm if it exists in algo_list, else print error message and exit
    for elem in algo_list:
        if elem[0] == algorithm_name:
            alg = elem[1](config)
            return alg

    raise ValueError(f"{algo_list.alg} is an invalid algorithm, list of valid argument values: {algo_list}")
