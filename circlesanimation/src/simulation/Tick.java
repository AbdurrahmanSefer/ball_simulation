package simulation;

public class Tick extends AbstractEvent {

    // Constructor : creates tick with a time
    // @param time the time of the tick
    public Tick(double time) {

        super(time);
    }

    // Checks validity of tick
    // Returns true always
    public boolean isValid() {
        
        return true;
    }

    // Make the tick happen signalling to the particle event handler
    // @param h the particle event handler
    public void happen(ParticleEventHandler h) {

        h.reactTo(this);
    }

}
