#include <boost/statechart/state_machine.hpp>
#include <boost/statechart/state.hpp>
#include <boost/statechart/transition.hpp>
#include <iostream>
#include <boost/mpl/list.hpp>
namespace mpl = boost::mpl;
namespace sc = boost::statechart;

struct FindObject: sc::event<FindObject> {
};
struct pickObject: sc::event<pickObject> {
};
struct findDestination: sc::event<findDestination> {
};
struct placeObject: sc::event<placeObject> {
};
struct goHome: sc::event<goHome> {
};
struct waitForCommand: sc::event<waitForCommand> {
};
struct error: sc::event<error> {
};

struct Active;
struct Wait;
struct Navigate;
struct Pick;
struct Place;
struct ErrorState;
struct PickupMachine: sc::state_machine<PickupMachine, Active> {

};

struct Active: sc::simple_state<Active, PickupMachine, Wait> {
	Active() {
		std::cout << "\nPR2 Robot is entered into Active State";
	}

	~Active() {
		std::cout << "\nPR2 Robot is leaving from Active State\n";
	}
};

struct Wait: sc::simple_state<Wait, Active> {
	Wait() {
		std::cout << "\nI am in wait state";
	}

	typedef sc::transition<FindObject, Navigate> reactions;
};
struct Navigate: sc::simple_state<Navigate, Active> {

	Navigate() {
		std::cout << "\nI am in Navigate state";
	}

	typedef mpl::list<
			sc::transition< pickObject, Pick >,
			sc::transition< placeObject, Place >,
			sc::transition< waitForCommand, Wait >,
			sc::transition< error, ErrorState > > reactions;


	/*typedef sc::transition<pickObject, Pick> reactions;
	typedef sc::transition<placeObject, Place> reaction2;
	typedef sc::transition<waitForCommand, Wait> reaction3;
	typedef sc::transition<error, ErrorState> reaction4;*/
};

struct Pick: sc::simple_state<Pick, Active> {
	Pick() {
		std::cout << "\nI am in Pick state";
	}
	typedef sc::transition<findDestination, Navigate> reactions;
};
struct Place: sc::simple_state<Place, Active> {
	Place() {
		std::cout << "\nI am in Place state";
	}
	typedef sc::transition<goHome, Navigate> reactions;
};
struct ErrorState: sc::simple_state<ErrorState, Active> {
	ErrorState() {
		std::cout << "\nI am in ErrorState state";
	}
};


/*int main() {
	std::cout << "\nBuilding a state machine";
	PickupMachine pm;
	pm.initiate();
	pm.process_event(FindObject());
	pm.process_event(pickObject());
	pm.process_event(findDestination());
	pm.process_event(placeObject());
	pm.process_event(goHome());
	return 0;
}*/
