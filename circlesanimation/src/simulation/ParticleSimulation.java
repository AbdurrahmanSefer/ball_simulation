package simulation;

import java.lang.reflect.InvocationTargetException;

import javax.swing.SwingUtilities;
import utils.MinPriorityQueue;

public class ParticleSimulation implements Runnable, ParticleEventHandler {

    private static final long FRAME_INTERVAL_MILLIS = 40;

    private final ParticlesModel        model;
    private final ParticlesView         screen;
    private MinPriorityQueue<Event>     queue;
    private double                      clock;

    // Constructor: creates a particle simulation
    // @param name the name of the simulation
    // @param m the particles model
    public ParticleSimulation(String name, ParticlesModel m) {

        model = m;
        screen = new ParticlesView(name, m);
        queue = new MinPriorityQueue<Event>();

        // Add a new 1.0 Tick
        clock = 1.0;
        queue.add(new Tick(clock));

        // Add all collisions to the queue
        Iterable<Collision> predicted_iterable = 
        model.predictAllCollisions(clock);

        for (Collision collision : predicted_iterable) {
            
            queue.add(collision);
        }
    }

    // Runs the simulation
    @Override
    public void run() {

        try {
            SwingUtilities.invokeAndWait(screen);
        } catch (InvocationTargetException e) {
            e.printStackTrace();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        // Repeat loop until queue is empty
        while (!(queue.isEmpty())) {

            // Remove the root and check validity
            Event root = this.queue.remove();
            if (root.isValid()) {

                // Move the particles for the time elapsed since the previous
                // event
                model.moveParticles(root.time() - this.clock);

                // Update the current time to the time of the event
                this.clock = root.time();
                root.happen(this);
            }

        }
    }

    public void reactTo(Tick t) {

        try {

            // Pause
            Thread.sleep(FRAME_INTERVAL_MILLIS);
        } catch (InterruptedException e) {

            e.printStackTrace();
        }
        // Update display and add new Tick to queue
        screen.update();
        queue.add(new Tick(clock + 1.0));
    }

    public void reactTo(Collision c) {

        Particle[] particles = c.getParticles();

        // Add new collisions to the queue for the particles that just collided
        for (Particle particle : particles) {

            Iterable<Collision> collision_iterable = 
            model.predictCollisions(particle, this.clock);

            for (Collision new_collision : collision_iterable) {

                this.queue.add(new_collision);
            }
        }
    }

}
