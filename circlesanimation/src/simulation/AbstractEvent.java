package simulation;

public abstract class AbstractEvent implements Event {

    protected double    time;

    // Constructor : creates an abstract event with a time
    // @param time the time of the event
    public AbstractEvent(double time) {
        
        this.time = time;
    }

    // Returns the time of the event
    @Override
    public double time() {
        
        return time;
    }

    // Compares event to another event based on their times
    // @param that the other event
    // Returns 1 if event is greater than other event
    // -1 if event is smaller than other event
    // 0 if the events are equal
    @Override
    public int compareTo(Event that) {
        
        if (time() > that.time()) {

            return 1;
        }

        if (time() < that.time()) {

            return -1;
        }

        return 0;
    }

}
