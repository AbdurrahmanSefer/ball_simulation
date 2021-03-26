package simulation;

public class TwoParticleCollision extends Collision{

    // Constructor : creates a two particle collision
    // @param p1 the first particle
    // @param p2 the second particle
    // @param t the time of the collision
    public TwoParticleCollision(Particle p1, Particle p2, double t) {

        super(t, new Particle[] {p1, p2});
    }

    // Make the collision happen and send signal the particle event handler
    // @param h the particle event handler
    public void happen(ParticleEventHandler h) {

        final Particle particles[] = getParticles();
        Particle.collide(particles[0], particles[1]);
        h.reactTo(this);
    }
}