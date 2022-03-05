#Dijkstra PathPlanning  
This code is required numpy, cv2, ast, math to run  
To run the code, simply type  
```bash  
python Dijkstra.py  
```
Then type the start point (x, y)  
For example, "50 150" in this case  
Then type the goal point (x, y)  
For example, "120 150" in this case  
If the points are out of boundary or in the obstacle, it will request the user to type again.  
After generating the graph, it will show "start dijkstra".  
When the point robot reaches the goal, it will show "at goal".  
Then it shows the time cost, path, and iterating cost.  
After seeing "success", there will be three images, two txt files, and one video.    
Images show the original image, added clearance image, and path image.  
The exploration.txt shows every points during the algorithm.  
The shortest_path.txt shows every points that Dijkstra finds the fastest path.  
The video performs the process how Dijkstra explores the image.  

