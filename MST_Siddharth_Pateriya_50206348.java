/*
 Siddharth Pateriya - Person Number 50206348
 UB CSE-531 - Fall 2016 ; Project 1
 */


/**
 *
 * @author Siddharth1
 */
import java.util.*;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.BufferedWriter;
import java.io.UnsupportedEncodingException;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.io.FileOutputStream;

/*
This is a Java program to output a Minimum Spanning Tree for a graph given in a file input.txt, having V vertices and E edges. Each edge is assigned two
vertices, u and v, and a cost, c. The edges are then added to a MinHeap and using Prim's algorithm, we find the Minimum Spanning Tree for the
given Graph.
*/


//Class for the Heap Object ( Min Heap)
class HeapObj {
    private ArrayList<Edge> Graph;

    public HeapObj() {
        Graph = new ArrayList<Edge>();
    }
    //Function to retrieve the top element in the heap, but not delete it
     public Edge peek() {
        return Graph.get(0);
    }
     //Function to insert an Edge e to the heap
      public void insert(Edge e) {
        Graph.add(e);
        HeapUp();
    }

    //Function to balance the heap after adding a new Edge to it
    private void HeapUp() {
        int j = Graph.size() - 1;
        while (j > 0) {
            int k = (j - 1) / 2;
            Edge current = Graph.get(j);
            Edge parent = Graph.get(k);
            if (current.cost < parent.cost) {
                Graph.set(j, parent);
                Graph.set(k, current);
                j = k;
            } else {
                break;       }  }    }

//Function to extract the top most element in the heap, 
//remove it from the heap, and balance the heap afterwards
    public Edge extmin()
  {      
        if (Graph.size() == 1) {
            return Graph.remove(0);
        }
        Edge min = Graph.get(0);
        Graph.set(0, Graph.remove(Graph.size() - 1));
        HeapDown();
        return min;
    }

//Function to balance the heap after extracting the top most element 
    private void HeapDown() {
        int j = 0;
        int l = 2 * j + 1;
        while (l < Graph.size()) {
            int max = l, r = l + 1;
            if (r < Graph.size()) { // there is a right child
                if (Graph.get(r).cost < Graph.get(l).cost) {
                    max++;} }
            if (Graph.get(j).cost > Graph.get(max).cost) {
                Edge tmp = Graph.get(j);
                Graph.set(j, Graph.get(max));
                Graph.set(max, tmp);
                j = max;
                l = 2 * j + 1;
            } else {
                break;
            }
        }
    }
  
  //Check if Heap is Empty
    public boolean isEmpty() {
        return Graph.isEmpty();

    }

   }


//Custom Class Edge which has two vertices u and v and a cost c
     class Edge {

        int u, v, cost;
        Edge(int from, int to, int c) {
            u = from;
            v = to;
            cost = c;
        }
    }

public class MST_Siddharth_Pateriya_50206348 {

     
    public static void main(String[] args) throws FileNotFoundException, UnsupportedEncodingException, IOException {

        Scanner in = new Scanner(new File("input.txt"));        //Read input.txt, which is the file containing the graph
        String ofile = "output.txt";                            // Set output file name
        try (BufferedWriter bw = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(ofile)))) {
            int numVert;            //Integer to store number of vertices
            int numEdge;            //Integer to store number of edges
            numVert = in.nextInt();
            numEdge = in.nextInt();
                       
            Edge[] edges = new Edge[numEdge];                //Create an array of Edges and add all edges sequentially to the array
            for (int i = 0; i < numEdge; i++) {
                int u = in.nextInt();
                int v = in.nextInt();
                int cost = in.nextInt();
                edges[i] = new Edge(u, v, cost);
            }
            
            ArrayList<ArrayList<Edge>> graph = MakeGraph(edges);    //Create a graph from the edges
            
            ArrayList<Edge> mstree = GetMSTPrim(graph);             // Create the MST from the graph
            
            int mst_cost = 0;                                       //Set initial cost = 0
            int tree_size = mstree.size();                          
                        
            for (int f = 0; f < tree_size; f++) {                   //Calculate cost of all edges in MST
                mst_cost = mst_cost + mstree.get(f).cost;}
            bw.write(Integer.toString(mst_cost));
            bw.newLine();
            System.out.println(mst_cost);            
            
            for (int f = 0; f < tree_size; f++)                     //Output all edges from the MST
            {
                bw.write( mstree.get(f).u + " " + mstree.get(f).v
                        + " " + mstree.get(f).cost);
                bw.newLine(); }
        }
    }
//Function for finding out minimum spanning tree using Prim's Algorithm
    public static ArrayList<Edge> GetMSTPrim(ArrayList<ArrayList<Edge>> Graph) {
        if (Graph.isEmpty()) 
        { throw new NullPointerException("Empty Graph");
        }
        HeapObj H1 = new HeapObj();
        ArrayList<Edge> mstree = new ArrayList<>();
        
        //for (int i = 0;i<Graph.size();i++){
        for (Edge e : Graph.get(1)) {
          H1.insert(new Edge(e.u, e.v, e.cost));            //Insert edges of first vertex to the heap
        }
        //}
        boolean[] visited = new boolean[Graph.size()];     //Array to store whether a vertex has been visited already or not
        visited[0] = true;
        while (!H1.isEmpty()) {                            //Run as long as there are edges in Heap
            Edge e = H1.extmin();                           //Extract Min edge from Heap H1
            if (visited[e.u] && visited[e.v])               //If both vertices have already been visited, move to the next edge
            { continue; }
            visited[e.u] = true;                            //Set vertex visited = true
            for (Edge edge : Graph.get(e.v)) {              //Insert edges of next vertex in the heap
                if (!visited[edge.v]) {
                    H1.insert(edge);                        //Insert edge if vertex not already visited
                }
            }
            visited[e.v] = true;                            //Set vertex as visited
            mstree.add(e);}                                 //Add edge to the Minimum Spanning Tree
        
        return mstree;
    }
    
//Function for creating the graph from the edges extracted from the input file
    public static ArrayList<ArrayList<Edge>> MakeGraph(Edge[] edges) {
        ArrayList<ArrayList<Edge>> Graph = new ArrayList<>();
        int length = 2 * edges.length;                          //Multiply by two since for each edge there also exists a reverse edge
        for (int i = 0; i < length; ++i) {
            Graph.add(new ArrayList<>());                       //Add empty lists to graph, equal to the number of edges (including reverse) 
        }

        for (Edge e : edges) {                                  //Add both forward and reverse edge to the Graph
            Edge reverse = new Edge(e.v, e.u, e.cost);
            Graph.get(e.u).add(e);
            Graph.get(e.v).add(reverse);           
        }

        return Graph;
    }

}


