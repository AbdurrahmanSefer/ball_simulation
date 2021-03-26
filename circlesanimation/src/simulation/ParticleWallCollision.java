package simulation;

public class ParticleWallCollision extends Collision{
    
    private Wall        wall;
    
    // Constructor : creates a collision between a particle and a wall
    // @param p the particle
    // @param w the wall
    // @param t the time of the collision
    public ParticleWallCollision(Particle p, Wall w, double t) {

        super(t, new Particle[] {p});
        wall = w;
    }

    // Make the collision happen and send signal to particle event handler
    // @param h the particle event handler
    public void happen(ParticleEventHandler h) {

        final Particle particles[] = getParticles();
        Particle.collide(particles[0], wall);
        h.reactTo(this);
    }
}