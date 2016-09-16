// Copyright 2010 Christophe Henry
// henry UNDERSCORE christophe AT hotmail DOT com
// This is an extended version of the state machine available in the boost::mpl library
// Distributed under the same license as the original.
// Copyright for the original version:
// Copyright 2005 David Abrahams and Aleksey Gurtovoy. Distributed
// under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#include <iostream>
// back-end
#include <boost/msm/back/state_machine.hpp>
//front-end
#include <boost/msm/front/state_machine_def.hpp>

namespace msm = boost::msm;
namespace mpl = boost::mpl;

namespace sm
{
    // events
    struct activate {};
    struct navigate {};
    //struct objectNotFound{};
    struct wait {};
    //struct pick {};
    struct place {};
    struct error {};


    // A "complicated" event type that carries some data.
	enum ObjectTypeEnum
    {
        COKE=0,
        BOTTLE=1,
		BOX=2
    };
    struct pick
    {
        pick(std::string name, ObjectTypeEnum objectType): name(name),object_type(objectType){
        }

        std::string name;
        ObjectTypeEnum object_type;
    };

    // front-end: define the FSM structure
    struct pickupMachine_ : public msm::front::state_machine_def<pickupMachine_>
    {
        template <class Event,class FSM>
        void on_entry(Event const& ,FSM&)
        {
            std::cout << "entering: PickupMachine" << std::endl;
        }
        template <class Event,class FSM>
        void on_exit(Event const&,FSM& )
        {
            std::cout << "leaving: PickupMachine" << std::endl;
        }

        // The list of FSM states
        struct Empty : public msm::front::state<>
        {
            // every (optional) entry/exit methods get the event passed.
            template <class Event,class FSM>
            void on_entry(Event const&,FSM& ) {std::cout << "\n entering: Empty" << std::endl;}
            template <class Event,class FSM>
            void on_exit(Event const&,FSM& ) {std::cout << "\n leaving: Empty" << std::endl;}
        };
        struct Waiting : public msm::front::state<>
        {
            template <class Event,class FSM>
            void on_entry(Event const& ,FSM&) {std::cout << "\n entering: Waiting" << std::endl;}
            template <class Event,class FSM>
            void on_exit(Event const&,FSM& ) {std::cout << "\n leaving: Waiting" << std::endl;}
        };

        // sm_ptr still supported but deprecated as functors are a much better way to do the same thing
        struct Picking : public msm::front::state<>
        {
            template <class Event,class FSM>
            void on_entry(Event const& ,FSM&) {std::cout << "\n entering: Picking" << std::endl;}
            template <class Event,class FSM>
            void on_exit(Event const&,FSM& ) {std::cout << "\n leaving: Picking" << std::endl;}
            /*void set_sm_ptr(pickupMachine_* pl)
            {
                m_pickupMachine=pl;
            }
            pickupMachine_* m_pickupMachine;*/
        };

        struct Failure : public msm::front::state<> {
		  template <class Event,class FSM>
		  void on_entry(Event const& ,FSM&) {std::cout << "\n entering: Failure" << std::endl;}
		  template <class Event,class FSM>
		  void on_exit(Event const&,FSM& ) {std::cout << "\n leaving: Failure" << std::endl;}
	    };

        struct Placing : public msm::front::state<> {
		  template <class Event,class FSM>
		  void on_entry(Event const& ,FSM&) {std::cout << "\n entering: Placing" << std::endl;}
		  template <class Event,class FSM>
		  void on_exit(Event const&,FSM& ) {std::cout << "\n leaving: Placing" << std::endl;}
		};

        struct Navigating : public msm::front::state<>
        {
            template <class Event,class FSM>
            void on_entry(Event const&,FSM& ) {std::cout << "\n entering: Navigating" << std::endl;}
            template <class Event,class FSM>
            void on_exit(Event const&,FSM& ) {std::cout << "\n leaving: Navigating" << std::endl;}
        };

        // state not defining any entry or exit
        struct Paused : public msm::front::state<>
        {
        };

        // the initial state of the player SM. Must be defined
        typedef Empty initial_state;

        // transition actions
        void searchObject(navigate const&)       { std::cout << "pickupMachine::searchObjects\n"; }
        void emptyObjects(wait const&)    { std::cout << "pickupMachine::emptyObjects\n"; }
        void pickupObject(pick const&)   { std::cout << "pickupMachine::pickupObject\n"; }
        void findDestination(navigate const&) { std::cout << "pickupMachine::findDestination\n"; }
        void placeObject(place const&)        { std::cout << "pickupMachine::placeObject\n"; }
        void goHome(navigate const&)      { std::cout << "pickupMachine::goHome\n"; }
        void reportFailure(error const&)      { std::cout << "pickupMachine::reportFailure\n"; }
        void initializeRobot(activate const&) {std::cout<<"pickupMachine::initializeRobot\n";}
        // guard conditions
        bool isObjectFound(pick const& evt)
        {
            // to test a guard condition, let's say we understand only COKE, not BOTTLE and BOX
            if (evt.object_type != COKE)
            {
                std::cout << "Object not found, sorry" << std::endl;
                return false;
            } else {
            	std::cout<< "Object found"<<std::endl;
            }
            return true;
        }
        // used to show a transition conflict. This guard will simply deactivate one transition and thus
        // solve the conflict
        bool auto_start(pick const&)
        {
            return false;
        }

        typedef pickupMachine_ p; // makes transition table cleaner

        // Transition table for player
        struct transition_table : mpl::vector<
            //    Start     Event             Next         Action				 Guard
            //  +---------+-------------+---------+---------------------+----------------------+
          a_row < Picking 	 , navigate     , Navigating , &p::findDestination                          >,
          a_row < Picking 	 , error        , Failure    , &p::reportFailure                            >,
            //  +---------+-------------+---------+---------------------+----------------------+
          a_row < Placing 	 , navigate     , Navigating , &p::goHome                                   >,
            //  +---------+-------------+---------+---------------------+----------------------+
          a_row < Empty   	 , activate     , Waiting    , &p::initializeRobot                          >,
            //  +---------+-------------+---------+---------------------+----------------------+
          a_row < Waiting 	 , navigate     , Navigating , &p::searchObject                             >,
		  a_row < Waiting 	 , error        , Failure    , &p::reportFailure                            >,
		    //  +---------+-------------+---------+---------------------+----------------------+
		  a_row < Navigating , navigate     , Navigating , &p::searchObject                             >,
          a_row < Navigating , error        , Failure    , &p::reportFailure                            >,
            row < Navigating , pick         , Picking 	 , &p::pickupObject     , &p::isObjectFound     >,
          a_row < Navigating , place        , Placing 	 , &p::placeObject                              >,
          a_row < Navigating , wait 		, Waiting    , &p::emptyObjects                             >
            //  +---------+-------------+---------+---------------------+----------------------+
        > {};
        // Replaces the default no-transition response.
        template <class FSM,class Event>
        void no_transition(Event const& e, FSM&,int state)
        {
            std::cout << "no transition from state " << state
                << " on event " << typeid(e).name() << std::endl;
        }
    };
    // Pick a back-end
    typedef msm::back::state_machine<pickupMachine_> pickupMachine;

    //
    // Testing utilities.
    //
    static char const* const state_names[] = { "Picking", "Placing", "Empty", "Waiting", "Navigating",   "Failure" };
    void pstate(pickupMachine const& p)
    {
    	std::cout << " -> " <<p.current_state()[0] << std::endl;
        std::cout << " -> " << state_names[p.current_state()[0]] << std::endl;
    }


}
