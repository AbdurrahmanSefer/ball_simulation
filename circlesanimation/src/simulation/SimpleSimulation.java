package simulation;

import java.awt.Color;
import java.util.Random;

public class SimpleSimulation {

    private static int dairelerin_sayisi = 5;
    
    public static void main(String[] args) {
        int w  = 600;
        int h  = 600;
        Random rand = new Random();
        Particle[] ps = new Particle[dairelerin_sayisi];                         
        for (int i = 0; i < dairelerin_sayisi; i++) {
            int r_=(int)(Math.random() * ((90 - 30) + 1)) + 30;
            int x = (int) (Math.random() * (w - r_)
                    + r_ / 2);
            int y = (int) (Math.random() * (w - r_)
                    + r_ / 2);
            double dx = 0, dy = 0;
            while (dx == 0) {
                dx = Math.random() * 11 - 5;
            }
            while (dy == 0) {
                dy = Math.random() * 11 - 5;
            }
            float r = rand.nextFloat();
            float g = rand.nextFloat();
            float b = rand.nextFloat();
            
            
            Color randomColor = new Color(r, g, b);
            
         
            ps[i] = new Particle(x, y, dx, dy, r_, 1, randomColor);
        }
        
        ParticlesModel     model = new ParticlesModel(w, h, ps);
        ParticleSimulation sim = 
                new ParticleSimulation("Elastic Collisions in 2D", model);        
        sim.run();
    }

}


