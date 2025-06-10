package bcc.javaJostle;

import java.util.ArrayList;
public class MyRobot extends Robot{
    // 0 - rock; 1 - random; 2 - opponent
    private int gmode = 0;
    private int[] targetPos;
    private ArrayList<Move> path = new ArrayList<>();
    private int stepX = -1;
    private int stepY = -1;
    private int startTileX;
    private int startTileY;
    private Move movement = new Move(0, 0);

    public MyRobot(int x, int y){
        super(x, y, 3, 2, 2, 3,"bob", "myRobotImage.png", "defaultProjectileImage.png");
        // Health: 3, Speed: 3, Attack Speed: 2, Projectile Strength: 2
        // Total = 10
        // Image name is "myRobotImage.png"
    }

    public void think(ArrayList<Robot> robots, ArrayList<Projectile> projectiles, Map map, ArrayList<PowerUp> powerups) {
        if(gmode == 0 && targetPos == null) {
            targetPos = new int[2];
            for(Robot r : robots) {
                if (r.getName().equals(getName())) continue;
                if (r.getName().equals("Rando")) {
                    gmode = 1;
                } else if (r.getName().equals("Rock")) {
                    gmode = 0;
                } else {
                    gmode = 2;
                }
                targetPos[0] = r.getX();
                targetPos[1] = r.getY();
            }
            startTileX = getX() / Utilities.TILE_SIZE;
            startTileY = getY() / Utilities.TILE_SIZE;
            path = findPath(getX() / Utilities.TILE_SIZE, getY() / Utilities.TILE_SIZE, targetPos[0] / Utilities.TILE_SIZE, targetPos[1] / Utilities.TILE_SIZE, map.getTiles());
        }
        if(gmode == 1) {
            for(Robot r : robots) {
                if(r.getName().equals("Rando")) {
                    targetPos[0] = r.getX();
                    targetPos[1] = r.getY();
                }
            }
        }
        if(gmode == 2) {
            for(Robot r : robots) {
                if(r.getName().equals(getName())) continue;
                targetPos[0] = r.getX();
                targetPos[1] = r.getY();
                break;
            }
        }

        if(!path.isEmpty()) {
            Move nextMove = path.get(0);
            int dx = nextMove.getX();
            int dy = nextMove.getY();
            
            if(dx == 0) {
                if(stepY == -1) {
                    movement.setY(movement.getY() + dy);
                    stepY = (startTileY + movement.getY()) * Utilities.TILE_SIZE;
                }
                if((dy > 0 && getY() >= stepY) || (dy < 0 && getY() <= stepY)) {
                    path.remove(0);
                    stepY = -1;
                } else {
                    yMovement = dy;
                }
                if(canAttack()) {
                    shootAtLocation(targetPos[0], targetPos[1]);
                }
                return;
            } else if(dy == 0) {
                if(stepX == -1) {
                    movement.setX(movement.getX() + dx);
                    stepX = (startTileX + movement.getX()) * Utilities.TILE_SIZE;
                }
                if((dx > 0 && getX() >= stepX) || (dx < 0 && getX() <= stepX)) {
                    path.remove(0);
                    stepX = -1;
                } else {
                    xMovement = dx;
                }
                if(canAttack()) {
                    // If we can't find a path, shoot at the target
                    shootAtLocation(targetPos[0] + Utilities.ROBOT_SIZE/2, targetPos[1] + Utilities.ROBOT_SIZE/2);
                }
                return;
            }
        } else if(gmode == 1 || gmode == 2) {
            for(Robot r : robots) {
                if(r.getName().equals("Rando")) {
                    targetPos[0] = r.getX();
                    targetPos[1] = r.getY();
                }
            }
            path = findPath(getX() / Utilities.TILE_SIZE, getY() / Utilities.TILE_SIZE, targetPos[0] / Utilities.TILE_SIZE, targetPos[1] / Utilities.TILE_SIZE, map.getTiles());
        }
        if(canAttack()) {
            // If we can't find a path, shoot at the target
            shootAtLocation(targetPos[0] + Utilities.ROBOT_SIZE/2, targetPos[1] + Utilities.ROBOT_SIZE/2);
        }
       /* Implement your robot's logic here
         For example, you can move towards the nearest robot or shoot at a target
         to move, choose a direciton to go
         to move left - use xMovement = -1
         to move right - use xMovement = 1
         to move up - use yMovement = -1
         to move down - use yMovement = 1
         You can ONLY move in one direction at a time, if your output doesn't match the above you will get an error

         to shoot, use shootAtLocation(x, y) where x and y are the coordinates of the target
         only shoot when canAttack() is true!
        */
       // System.out.println("Thinking...");
       
    }

    public ArrayList<Move> findPath(int fromx, int fromy, int tox, int toy, int[][] map) {
        ArrayList<Node> open = new ArrayList<>();
        boolean[][] closed = new boolean[map.length][map[0].length];

        open.add(new Node(fromx, fromy, 0, heuristic(fromx, fromy, tox, toy), null));

        while(!open.isEmpty()) {
            Node current = open.get(0);
            for(Node node : open) {
                if (node.f() < current.f()) {
                    current = node;
                }
            }

            open.remove(current);

            if(current.x == tox && current.y == toy) {
                // Path found
                ArrayList<Move> path = new ArrayList<>();
                ArrayList<Move> finalPath = new ArrayList<>();
                while(current.parent != null) {
                    path.add(new Move(current.x - current.parent.x, current.y - current.parent.y));
                    current = current.parent;
                }
                // Reverse the path to get it from start to goal
                for(int i = path.size() - 1; i >= 0; i--) {
                    finalPath.add(path.get(i));
                }
                finalPath.remove(finalPath.size() - 1);
                return finalPath; // Exit after finding the path
            }

            closed[current.y][current.x] = true;

            // Check neighbors
            for(int dx = -1; dx <= 1; dx++) {
                for(int dy = -1; dy <= 1; dy++) {
                    if(Math.abs(dx) == Math.abs(dy)) continue; // Skip diagonals
                    int nx = current.x + dx;
                    int ny = current.y + dy;
                    if(inBounds(nx, ny, map[0].length, map.length) && map[ny][nx] != Utilities.WALL && !closed[ny][nx]) {
                        boolean skip = false;
                        for (Node n : open) {
                            if (n.x == nx && n.y == ny && n.g <= current.g + 1) {
                                skip = true; // Already in open list
                                break;
                            }
                        }
                        if(!skip) {
                            int cost = current.g + 1;
                            int h = heuristic(nx, ny, tox, toy);
                            if (map[ny][nx] == Utilities.MUD) {
                                cost += 1; // Mud increases cost
                            }
                            // Walls are already handled by inBounds check
                            Node neighbor = new Node(nx, ny, cost, h, current);
                            open.add(neighbor);
                        }
                    }
                }
            }
        }
        return new ArrayList<>(); // No path found
    }

    private class Move {
        int dx;
        int dy;

        Move(int dx, int dy) {
            this.dx = dx;
            this.dy = dy;
        }

        public int getX() {
            return dx;
        }

        public int getY() {
            return dy;
        }

        private int setX(int dx) {
            this.dx = dx;
            return this.dx;
        }
        private int setY(int dy) {
            this.dy = dy;
            return this.dy;
        }

        @Override
        public String toString() {
            return "Move{" + "dx=" + dx + ", dy=" + dy + '}';
        }
    }

    private class Node {
        int x;
        int y;
        int g; // Cost from start to this node
        int h; // Heuristic cost to goal
        Node parent;

        Node(int x, int y, int g, int h, Node parent) {
            this.x = x;
            this.y = y;
            this.g = g;
            this.h = h;
            this.parent = parent;
        }

        public int f() {
            return g + h; // Total cost
        }
    }

    private static int heuristic(int x1, int y1, int x2, int y2) {
        // Manhattan distance
        return Math.abs(x1 - x2) + Math.abs(y1 - y2);
    }

    private static boolean inBounds(int x, int y, int cols, int rows) {
        return x >= 0 && x < cols && y >= 0 && y < rows;
    }
}
