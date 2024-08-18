#!/usr/bin/python3

from graphviz import Graph
import json
import argparse


def create_graph(edges_with_costs,node_colors ):
    # Create an undirected graph
    g = Graph('G', format='png')

    # Add all nodes first to ensure they are defined before adding edges
    nodes = set()
    for start, end, _ in edges_with_costs:
        nodes.add(start)
        nodes.add(end)

    # Add nodes with specific colors
    for node in nodes:
        color = node_colors.get(node, None)
        if color:
            g.node(node, color=color, style='filled')  # Set color if specified
        else:
            g.node(node)  # Add node without specific color

    # Add edges after nodes are defined
    for start, end, cost in edges_with_costs:
        g.edge(start, end, label=f"{str(cost)}")

    # Save the graph to a PNG file
    g.render('graph')

def main(_dict=None, json_file=None):
    if json_file and _dict:
        print("ERROR - expected one argument")
        return False

    # Process the arguments
    if json_file:
        with open(json_file, 'r') as f:
            dictionary = json.load(f)
    else:
        dictionary = _dict

    start = dictionary['start']
    goal = dictionary['goal']
    path_nodes = dictionary['path_list'] #Example: [[94, 113], [99, 111], [121, 116], [132, 100], [160, 99], [182, 106]]
    path_edges = dictionary['path_edges']
    #Example: 
        # [
        #     [
        #         "(384, 74)",
        #         "(363, 70)",
        #         21.377558326431952
        #     ],
        #     [
        #         "(384, 74)",
        #         "(390, 74)",
        #         6.0
        #     ]
        # ]
    
    node_colors = {
        f'{start}': 'lightblue',     # Color for start node
        f'{goal}': 'lightgreen'     # Color for last node
    }

    for path in path_nodes:
        if list(path) != start and list(path) != goal:
            node_colors[f"{list(path)}"] = 'lightpink'

    create_graph(path_edges, node_colors)
    return True

# if __name__ == '__main__':
# # Create the parser
#     parser = argparse.ArgumentParser(description="command-line parser")
#
#     # Add arguments to the mutually exclusive group
#     # group = parser.add_mutually_exclusive_group(required=True)
#     parser.add_argument('--jsonFile', required=True, type=str, help='The path of the json dict file')
#     # group.add_argument('--dictionary', type=json.loads, help='A JSON-formatted dictionary string')
#
#     # Parse the arguments
#     args = parser.parse_args()
#
#     if args.jsonFile: main(json_file=args.jsonFile)
#     # main(_dict=args.dictionary)

my_dict = {
    "start": [99, 111],
    "goal": [182, 106],
    "path_list": [
        [99, 111],
        [121, 116],
        [132, 100],
        [160, 99],
        [182, 106]
    ],
    "path_edges": [
        [
            "[99, 111]",
            "[121, 116]",
            10.0
        ],
        [
            "[121, 116]",
            "[132, 100]",
            15.0
        ],
        [
            "[132, 100]",
            "[160, 99]",
            8.0
        ],
        [
            "[160, 99]",
            "[182, 106]",
            12.0
        ],
        [
            "[50, 50]",
            "[200, 200]",
            50.0
        ],
        [
            "[50, 50]",
            "[94, 113]",
            30.0
        ]
    ]
}

main(_dict=my_dict)
