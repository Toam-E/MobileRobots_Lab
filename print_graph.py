from graphviz import Graph
import json

def create_graph(edges_with_costs,node_colors ):
    # Create an undirected graph
    g = Graph('G', format='png')

    # Add all nodes first to ensure they are defined before adding edges
    nodes = set()
    for start, end, _, _ in edges_with_costs:
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
    for start, end, cost, h in edges_with_costs:
        g.edge(start, end, label=f"{str(cost)}, {str(h)}, {str(cost+h)}")

    # Save the graph to a PNG file
    g.render('graph')

def main():
    start = (94, 113)
    goal = tuple(json.load(open("/Users/toamelharar/Documents/GitHub/MobileRobots_HW2/hw2-wet2/goal.json", "r")))
    node_colors = {
        f'{start}': 'lightblue',     # Color for start node
        f'{goal}': 'lightgreen'     # Color for last node
    }
    path_list = json.load(open('/Users/toamelharar/Documents/GitHub/MobileRobots_HW2/hw2-wet2/path.json', 'r'))
    print(path_list)
    for path in path_list:
        if tuple(path) != start and tuple(path) != goal:
            node_colors[f"{tuple(path)}"] = 'lightpink'
    print(node_colors)
    edges_path = "/Users/toamelharar/Documents/GitHub/MobileRobots_HW2/hw2-wet2/AStar_tree/graph"
    with open(edges_path, 'r') as f:
        json_edges = json.load(f)
    create_graph(json_edges, node_colors)

if __name__ == '__main__':
    main()