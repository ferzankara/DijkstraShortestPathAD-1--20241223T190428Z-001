import java.util.*;

public class DijkstraShortestPath {
    private static class Node {
        private String name;
        private List<Edge> edges;

        public Node(String name) {
            this.name = name;
            this.edges = new ArrayList<>();
        }

        public void addEdge(Node destination, int weight) {
            edges.add(new Edge(destination, weight));
        }

        public String getName() {
            return name;
        }

        public List<Edge> getEdges() {
            return edges;
        }
    }

    private static class Edge {
        private Node destination;
        private int weight;

        public Edge(Node destination, int weight) {
            this.destination = destination;
            this.weight = weight;
        }

        public Node getDestination() {
            return destination;
        }

        public int getWeight() {
            return weight;
        }
    }

    private static Map<String, Node> graph;

    public static void main(String[] args) {
        buildGraph();
        if (args.length < 2) {
            System.out.println("Please provide source and target nodes as command line arguments.");
            return;
        }

        String sourceName = args[0];
        String targetName = args[1];

        Node source = graph.get(sourceName);
        Node target = graph.get(targetName);

        if (source == null || target == null) {
            System.out.println("Invalid source or target node.");
            return;
        }

        Map<Node, Integer> distances = new HashMap<>();
        Map<Node, Node> previousNodes = new HashMap<>();
        PriorityQueue<Node> queue = new PriorityQueue<>(Comparator.comparingInt(distances::get));

        for (Node node : graph.values()) {
            distances.put(node, Integer.MAX_VALUE);
            previousNodes.put(node, null);
        }

        distances.put(source, 0);
        queue.add(source);

        while (!queue.isEmpty()) {
            Node current = queue.poll();

            if (current == target) {
                break;
            }

            for (Edge edge : current.getEdges()) {
                Node neighbor = edge.getDestination();
                int weight = edge.getWeight();
                int distance = distances.get(current) + weight;

                if (distance < distances.get(neighbor)) {
                    distances.put(neighbor, distance);
                    previousNodes.put(neighbor, current);
                    queue.add(neighbor);
                }
            }
        }

        if (previousNodes.get(target) == null) {
            System.out.println("There is no path between " + sourceName + " and " + targetName + ".");
        } else {
            List<Node> path = new ArrayList<>();
            Node node = target;

            while (node != null) {
                path.add(0, node);
                node = previousNodes.get(node);
            }

            System.out.println("Shortest path between " + sourceName + " and " + targetName + ":");
            System.out.print("Shortest path: ");
            for (int i = 0; i < path.size() - 1; i++) {
                System.out.print(path.get(i).getName() + "-");
            }
            System.out.println(path.get(path.size() - 1).getName());

            System.out.println("Distance: " + distances.get(target));
        }
    }

    private static void buildGraph() {
        graph = new HashMap<>();

        Node A = new Node("A");
        Node B = new Node("B");
        Node C = new Node("C");
        Node D = new Node("D");
        Node E = new Node("E");
        Node F = new Node("F");
        Node G = new Node("G");
        Node H = new Node("H");
        Node I = new Node("I");
        Node J = new Node("J");
        Node K = new Node("K");
        Node L = new Node("L");
        Node M = new Node("M");
        Node O = new Node("O");
        Node P = new Node("P");
        Node Q = new Node("Q");
        Node R = new Node("R");
        Node S = new Node("S");
        Node T = new Node("T");
        Node U = new Node("U");
        Node V = new Node("V");
        Node X = new Node("X");
        Node Y = new Node("Y");
        Node Z = new Node("Z");

        A.addEdge(B, 3);
        A.addEdge(C, 4);
        A.addEdge(D, 2);
        B.addEdge(A, 3);
        B.addEdge(D, 5);
        C.addEdge(A, 4);
        C.addEdge(D, 5);
        C.addEdge(F, 6);
        D.addEdge(A, 2);
        D.addEdge(B, 5);
        D.addEdge(C, 5);
        D.addEdge(E, 1);
        E.addEdge(D, 1);
        F.addEdge(C, 6);
        F.addEdge(G, 4);
        G.addEdge(F, 4);
        G.addEdge(H, 2);
        G.addEdge(L, 5);
        H.addEdge(G, 2);
        H.addEdge(I, 3);
        H.addEdge(M, 2);
        I.addEdge(H, 3);
        I.addEdge(J, 4);
        J.addEdge(I, 4);
        J.addEdge(K, 2);
        K.addEdge(J, 2);
        K.addEdge(M, 6);
        L.addEdge(G, 5);
        L.addEdge(M, 2);
        M.addEdge(H, 2);
        M.addEdge(K, 6);
        M.addEdge(L, 2);
        O.addEdge(L, 4);
        O.addEdge(P, 3);
        O.addEdge(U, 5);
        P.addEdge(O, 3);
        P.addEdge(Q, 4);
        P.addEdge(V, 2);
        Q.addEdge(P, 4);
        Q.addEdge(R, 1);
        Q.addEdge(S, 3);
        R.addEdge(Q, 1);
        S.addEdge(Q, 3);
        S.addEdge(T, 2);
        T.addEdge(S, 2);
        U.addEdge(O, 5);
        U.addEdge(V, 3);
        V.addEdge(P, 2);
        V.addEdge(U, 3);
        X.addEdge(U, 2);
        X.addEdge(Y, 4);
        Y.addEdge(X, 4);
        Y.addEdge(Z, 3);
        Z.addEdge(Y, 3);

        graph.put("A", A);
        graph.put("B", B);
        graph.put("C", C);
        graph.put("D", D);
        graph.put("E", E);
        graph.put("F", F);
        graph.put("G", G);
        graph.put("H", H);
        graph.put("I", I);
        graph.put("J", J);
        graph.put("K", K);
        graph.put("L", L);
        graph.put("M", M);
        graph.put("O", O);
        graph.put("P", P);
        graph.put("Q", Q);
        graph.put("R", R);
        graph.put("S", S);
        graph.put("T", T);
        graph.put("U", U);
        graph.put("V", V);
        graph.put("X", X);
        graph.put("Y", Y);
        graph.put("Z", Z);
    }
}


