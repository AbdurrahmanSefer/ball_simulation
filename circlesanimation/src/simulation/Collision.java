package simulation;

public abstract class Collision extends AbstractEvent{
    
    public Particle[]   ps;
    public int[]        hits_at_t;

    // Constructor: creates a collision
    // @param t the time of the collision
    // @param ps the particle(s) involved
    public Collision(double t, Particle[] ps) {

        super(t);
        this.ps = ps;
        
        // Store number of hits at prediction time t
        hits_at_t = new int[ps.length];
        for (int i = 0; i < ps.length; ++i) {
            
            hits_at_t[i] = ps[i].collisions();
        }
    }


    // Checks if a collision is valid
    // Returns true if number of hits has not changed since t
    // false otherwise
    @Override
    public boolean isValid() {

        // Check if number of hits has changed since t
        for (int i = 0; i < this.ps.length; ++i) {

            if (hits_at_t[i] != ps[i].collisions()) {

                return false;
            }
        }

        return true;
    }


    // Returns particles involved in collision
    public Particle[] getParticles() {
        
        return ps;
    }
}
