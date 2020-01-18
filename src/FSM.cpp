#include "FSM.h"
#include <iostream>
using namespace std;

// Interpolation linéaire :
// Doit retourner la valeur dans l'intervalle [x1,x2] qui correspond au placement de y1 dans l'intervalle [0,y2]
float linear_interpolate (float x1, float x2, float y1, float y2) {
    /// ============ TODO PARTIE II ============= ///
    return x1*(y2-y1)/y2+x2*y1/y2;
    /// ========================================= ///
}

FSM::FSM() { }

void FSM::update(double Dt, bool lc, bool rc, std::vector<float> currentAnglesLocal, std::vector<float> currentAnglesGlobal) {

    // Mise à jour du temps écoulé dans l'état
    m_timeInState += Dt;
    // L'état courant
    State s = m_states[m_currentState];
    // Si la machine a un seul état on reste dedans
    if (m_nbStates==1) return;
    // Récupération des appuis courants
    ContactState footContact = NONE;
    if (lc && !rc) footContact = LEFT_SUPPORT;
    else if (!lc && rc) footContact = RIGHT_SUPPORT;
    else if (lc && rc) footContact = DOUBLE_SUPPORT;
    else if (!lc && !rc) footContact = NO_SUPPORT;
    // Transition si durée écoulée ou si appui correct
    if ((s.transitionOnTimeOrContact && (m_timeInState > s.transitionTime))
        || (!s.transitionOnTimeOrContact && (footContact == s.transitionContact))) {
        // Remise à zéro du temps écoulé
        m_timeInState = 0.0;
        // Récupération des angles courants en tant que point de départ de l'état (animation fluide)
        for (unsigned int i=0;i<s.targetAngles.size();i++) {
                if (m_states[s.nextState].targetLocal[i]) m_anglesAtTransition[i] = currentAnglesLocal[i];
                else m_anglesAtTransition[i] = currentAnglesGlobal[i];
        }
        // Passage à l'état suivant
        m_currentState = s.nextState;
    }
}

std::vector<float> FSM::getCurrentTargetAngles() const {
    // Poses clés non interpolées si 1 état ou transition directe ou basé appui
    if (m_nbStates==1 || !m_states[m_currentState].transitionOnTimeOrContact || m_states[m_currentState].transitionTime==0.0) return m_states[m_currentState].targetAngles;

    // Poses clés interpolées sinon
    std::vector<float> targetAnglesInterpolated;
    float y1 = m_timeInState; // durée écoulée depuis le début de l'état courant
    float y2 = m_states[m_currentState].transitionTime;  // durée max avant transition
    for (unsigned int i=0; i<m_states[m_currentState].targetAngles.size();i++) {
        float x1 = m_anglesAtTransition[i]; // l'angle au début de l'état
        float x2 = m_states[m_currentState].targetAngles[i]; // la cible courante
        if (x1==x2) targetAnglesInterpolated.push_back(x1); // pas d'interpolation si identiques
        else targetAnglesInterpolated.push_back(linear_interpolate(x1,x2,y1,y2)); // interpolation linéaire sinon
    }
    return targetAnglesInterpolated;
}

FSM_Stand::FSM_Stand() {
    // La machine à états finis pour un mouvement stable debout
    // 1 état, toutes les articulation à zéro dans le monde
    m_nbStates = 1;
    m_currentState = 0;
    State s;
    // ETAT 0 //
    s.ID = 0;
    s.nextState = 0;
    s.contactState = DOUBLE_SUPPORT;
    s.transitionOnTimeOrContact = true;
    s.transitionTime = 0.0;
    s.transitionContact = NONE;
    s.targetAngles.clear(); s.targetLocal.clear();
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //CHEVILLE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //CHEVILLE_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //GENOU_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //GENOU_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //HANCHE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //HANCHE_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //TRONC
    m_states.push_back(s);

}

FSM_Walk::FSM_Walk() {
    /// ============ TODO PARTIE II ============= ///
    // m_nbStates = ...; // Nombre d'états finis
    // m_currentState = ...; // Etat initial
    // State s; // L'état à ajouter
    // --- A répéter pour tous les états ---
    //    s.ID = ...; // Le numéro de l'état
    //    s.nextState = ...; // Le numéro de l'état suivant
    //    s.contactState = ...; // Le type de contact de l'état (optionnel)
    //    s.transitionOnTimeOrContact = ...; // true si transition basée sur le temps écoulé, faux si basé sur contact
    //    s.transitionTime = ...; // Temps de transition (si basé sur transition)
    //    s.transitionContact = ..; // Type de contact pour transition (si basé sur contact)
    //    s.targetAngles.clear(); s.targetLocal.clear();
    // --- --- A répéter pour toutes les articulations --- ---
    //       s.targetAngles.push_back(...); s.targetLocal.push_back(...); // Angles et indicateur global/local
    //    m_states.push_back(s);

    // copie des premières valeurs dans m_anglesAtTransition
    // for (unsigned int i=0;i<s.targetAngles.size();i++)
    //    m_anglesAtTransition.push_back(m_states[m_currentState].targetAngles[i]);
   /// ========================================== ///
    m_nbStates = 6;
    m_currentState = 0;
    State s;
    // ETAT 0 //
    s.ID = 0;
    s.nextState = 1;
    s.contactState = DOUBLE_SUPPORT;
    s.transitionOnTimeOrContact = true;
    s.transitionTime = 0.25;
    s.transitionContact = NONE;
    s.targetAngles.clear(); s.targetLocal.clear();
    s.targetAngles.push_back(-0.5); s.targetLocal.push_back(false);  //CHEVILLE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //CHEVILLE_DROIT
    s.targetAngles.push_back(-0.6); s.targetLocal.push_back(false);  //GENOU_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //GENOU_DROIT
    s.targetAngles.push_back(0.6); s.targetLocal.push_back(false);  //HANCHE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //HANCHE_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //TRONC
    m_states.push_back(s);
    // ETAT 1 //
    s.ID = 1;
    s.nextState = 2;
    s.contactState = DOUBLE_SUPPORT;
    s.transitionOnTimeOrContact = true;
    s.transitionTime = 0.25;
    s.transitionContact = NONE;
    s.targetAngles.clear(); s.targetLocal.clear();
    s.targetAngles.push_back(0.2); s.targetLocal.push_back(false);  //CHEVILLE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //CHEVILLE_DROIT
    s.targetAngles.push_back(0.2); s.targetLocal.push_back(false);  //GENOU_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //GENOU_DROIT
    s.targetAngles.push_back(0.9); s.targetLocal.push_back(false);  //HANCHE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //HANCHE_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //TRONC
    m_states.push_back(s);
    // ETAT 2 //
    s.ID = 2;
    s.nextState = 3;
    s.contactState = DOUBLE_SUPPORT;
    s.transitionOnTimeOrContact = true;
    s.transitionTime = 0.25;
    s.transitionContact = NONE;
    s.targetAngles.clear(); s.targetLocal.clear();
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //CHEVILLE_GAUCHE
    s.targetAngles.push_back(-0.6); s.targetLocal.push_back(false);  //CHEVILLE_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //GENOU_GAUCHE
    s.targetAngles.push_back(-0.9); s.targetLocal.push_back(false);  //GENOU_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //HANCHE_GAUCHE
    s.targetAngles.push_back(-0.7); s.targetLocal.push_back(false);  //HANCHE_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //TRONC
    m_states.push_back(s);
    // ETAT 3 //
    s.ID = 3;
    s.nextState = 4;
    s.contactState = DOUBLE_SUPPORT;
    s.transitionOnTimeOrContact = true;
    s.transitionTime = 0.25;
    s.transitionContact = NONE;
    s.targetAngles.clear(); s.targetLocal.clear();
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //CHEVILLE_GAUCHE
    s.targetAngles.push_back(-0.5); s.targetLocal.push_back(false);  //CHEVILLE_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //GENOU_GAUCHE
    s.targetAngles.push_back(-0.6); s.targetLocal.push_back(false);  //GENOU_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //HANCHE_GAUCHE
    s.targetAngles.push_back(0.6); s.targetLocal.push_back(false);  //HANCHE_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //TRONC
    m_states.push_back(s);
    // ETAT 4 //
    s.ID = 4;
    s.nextState = 5;
    s.contactState = DOUBLE_SUPPORT;
    s.transitionOnTimeOrContact = true;
    s.transitionTime = 0.25;
    s.transitionContact = NONE;
    s.targetAngles.clear(); s.targetLocal.clear();
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //CHEVILLE_GAUCHE
    s.targetAngles.push_back(0.2); s.targetLocal.push_back(false);  //CHEVILLE_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //GENOU_GAUCHE
    s.targetAngles.push_back(0.2); s.targetLocal.push_back(false);  //GENOU_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //HANCHE_GAUCHE
    s.targetAngles.push_back(0.9); s.targetLocal.push_back(false);  //HANCHE_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //TRONC
    m_states.push_back(s);
    // ETAT 5 //
    s.ID = 5;
    s.nextState = 0;
    s.contactState = DOUBLE_SUPPORT;
    s.transitionOnTimeOrContact = true;
    s.transitionTime = 0.25;
    s.transitionContact = NONE;
    s.targetAngles.clear(); s.targetLocal.clear();
    s.targetAngles.push_back(-0.6); s.targetLocal.push_back(false);  //CHEVILLE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //CHEVILLE_DROIT
    s.targetAngles.push_back(-0.9); s.targetLocal.push_back(false);  //GENOU_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //GENOU_DROIT
    s.targetAngles.push_back(-0.7); s.targetLocal.push_back(false);  //HANCHE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //HANCHE_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //TRONC
    m_states.push_back(s);


    // copie des premières valeurs dans m_anglesAtTransition
    for (unsigned int i=0;i<s.targetAngles.size();i++)
    {
        m_anglesAtTransition.push_back(m_states[0].targetAngles[i]);
    }
}


FSM_Run::FSM_Run() {
    /// ============ TODO PARTIE II ============= ///
    // m_nbStates = ...; // Nombre d'états finis
    // m_currentState = ...; // Etat initial
    // State s; // L'état à ajouter
    // --- A répéter pour tous les états ---
    //    s.ID = ...; // Le numéro de l'état
    //    s.nextState = ...; // Le numéro de l'état suivant
    //    s.contactState = ...; // Le type de contact de l'état (optionnel)
    //    s.transitionOnTimeOrContact = ...; // true si transition basée sur le temps écoulé, faux si basé sur contact
    //    s.transitionTime = ...; // Temps de transition (si basé sur transition)
    //    s.transitionContact = ..; // Type de contact pour transition (si basé sur contact)
    //    s.targetAngles.clear(); s.targetLocal.clear();
    // --- --- A répéter pour toutes les articulations --- ---
    //       s.targetAngles.push_back(...); s.targetLocal.push_back(...); // Angles et indicateur global/local
    //    m_states.push_back(s);

    // copie des premières valeurs dans m_anglesAtTransition
    // for (unsigned int i=0;i<s.targetAngles.size();i++)
    //    m_anglesAtTransition.push_back(m_states[m_currentState].targetAngles[i]);
   /// ========================================== ///
    m_nbStates = 4;
    m_currentState = 0;
    State s;
    // ETAT 0 //
    s.ID = 0;
    s.nextState = 1;
    s.contactState = DOUBLE_SUPPORT;
    s.transitionOnTimeOrContact = true;
    s.transitionTime = 0.2;
    s.transitionContact = NONE;
    s.targetAngles.clear(); s.targetLocal.clear();
    s.targetAngles.push_back(-0.4); s.targetLocal.push_back(false);  //CHEVILLE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //CHEVILLE_DROIT
    s.targetAngles.push_back(-0.4); s.targetLocal.push_back(false);  //GENOU_GAUCHE
    s.targetAngles.push_back(-0.2); s.targetLocal.push_back(false);  //GENOU_DROIT
    s.targetAngles.push_back(1.3); s.targetLocal.push_back(false);  //HANCHE_GAUCHE
    s.targetAngles.push_back(-0.1); s.targetLocal.push_back(false);  //HANCHE_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //TRONC
    m_states.push_back(s);
    // ETAT 1 //
    s.ID = 1;
    s.nextState = 2;
    s.contactState = DOUBLE_SUPPORT;
    s.transitionOnTimeOrContact = true;
    s.transitionTime = 0.2;
    s.transitionContact = NONE;
    s.targetAngles.clear(); s.targetLocal.clear();
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //CHEVILLE_GAUCHE
    s.targetAngles.push_back(-2.3); s.targetLocal.push_back(false);  //CHEVILLE_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //GENOU_GAUCHE
    s.targetAngles.push_back(-2.3); s.targetLocal.push_back(false);  //GENOU_DROIT
    s.targetAngles.push_back(0.7); s.targetLocal.push_back(false);  //HANCHE_GAUCHE
    s.targetAngles.push_back(-1.3); s.targetLocal.push_back(false);  //HANCHE_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //TRONC
    m_states.push_back(s);
    // ETAT 2 //
    s.ID = 2;
    s.nextState = 3;
    s.contactState = DOUBLE_SUPPORT;
    s.transitionOnTimeOrContact = true;
    s.transitionTime = 0.2;
    s.transitionContact = NONE;
    s.targetAngles.clear(); s.targetLocal.clear();
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //CHEVILLE_GAUCHE
    s.targetAngles.push_back(-0.4); s.targetLocal.push_back(false);  //CHEVILLE_DROIT
    s.targetAngles.push_back(-0.2); s.targetLocal.push_back(false);  //GENOU_GAUCHE
    s.targetAngles.push_back(-0.4); s.targetLocal.push_back(false);  //GENOU_DROIT
    s.targetAngles.push_back(-0.1); s.targetLocal.push_back(false);  //HANCHE_GAUCHE
    s.targetAngles.push_back(1.3); s.targetLocal.push_back(false);  //HANCHE_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //TRONC
    m_states.push_back(s);
    // ETAT 3 //
    s.ID = 3;
    s.nextState = 0;
    s.contactState = DOUBLE_SUPPORT;
    s.transitionOnTimeOrContact = true;
    s.transitionTime = 0.2;
    s.transitionContact = NONE;
    s.targetAngles.clear(); s.targetLocal.clear();
    s.targetAngles.push_back(-2.3); s.targetLocal.push_back(false);  //CHEVILLE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //CHEVILLE_DROIT
    s.targetAngles.push_back(-2.3); s.targetLocal.push_back(false);  //GENOU_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //GENOU_DROIT
    s.targetAngles.push_back(-1.3); s.targetLocal.push_back(false);  //HANCHE_GAUCHE
    s.targetAngles.push_back(0.7); s.targetLocal.push_back(false);  //HANCHE_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //TRONC
    m_states.push_back(s);


    // copie des premières valeurs dans m_anglesAtTransition
    for (unsigned int i=0;i<s.targetAngles.size();i++)
    {
        m_anglesAtTransition.push_back(m_states[0].targetAngles[i]);
    }
}


FSM_Walk_then_Run::FSM_Walk_then_Run() {

    m_FSM_Stander=new FSM_Stand();
    m_FSM_Walker=new FSM_Walk();
    m_FSM_Runner=new FSM_Run();

    m_nbStates=m_FSM_Stander->m_nbStates;
    m_currentState=m_FSM_Stander->m_currentState;
    m_states=m_FSM_Stander->m_states;
    m_timeInState=m_FSM_Stander->m_timeInState;
    m_anglesAtTransition=m_FSM_Stander->m_anglesAtTransition;
}

void FSM_Walk_then_Run::new_update(double Dt, bool lc, bool rc, std::vector<float> currentAnglesLocal, std::vector<float> currentAnglesGlobal) {


    m_all_time+=Dt;


    if(m_all_time<3)
    {
        m_FSM_Stander->update(Dt,lc,rc,currentAnglesLocal,currentAnglesGlobal);
    }
    else if(m_all_time<13)
    {
        for(int i=0;i<m_FSM_Walker->m_states.size();i++)
        {
            m_FSM_Walker->m_states[i].transitionTime=0.2+0.1*(13.0-m_all_time)/10.0;
        }
        m_FSM_Walker->update(Dt,lc,rc,currentAnglesLocal,currentAnglesGlobal);
    }
    else
    {
        if(m_all_time<23)
        {
            for(int i=0;i<m_FSM_Runner->m_states.size();i++)
            {
                m_FSM_Runner->m_states[i].transitionTime=0.18+0.1*(23.0-m_all_time)/10.0;
            }
        }
        if(m_FSM_Walker->m_currentState!=5)
        {
            m_FSM_Walker->update(Dt,lc,rc,currentAnglesLocal,currentAnglesGlobal);
        }
        else
        {
            m_FSM_Runner->update(Dt,lc,rc,currentAnglesLocal,currentAnglesGlobal);
        }
    }
}

std::vector<float> FSM_Walk_then_Run::getCurrentTargetAngles() const {

    if(m_all_time<3)
    {
        return m_FSM_Stander->getCurrentTargetAngles();
    }
    else if(m_all_time<13)
    {
        return m_FSM_Walker->getCurrentTargetAngles();
    }
    else
    {
        if(m_FSM_Walker->m_currentState!=5)
        {
            return m_FSM_Walker->getCurrentTargetAngles();
        }
        else
        {
            return m_FSM_Runner->getCurrentTargetAngles();
        }
    }
}

std::vector<bool> FSM_Walk_then_Run::getCurrentTargetLocal() const {

    if(m_all_time<3)
    {
        return m_FSM_Stander->getCurrentTargetLocal();
    }
    else if(m_all_time<13)
    {
        return m_FSM_Walker->getCurrentTargetLocal();
    }
    else
    {
        if(m_FSM_Walker->m_currentState!=5)
        {
            return m_FSM_Walker->getCurrentTargetLocal();
        }
        else
        {
            return m_FSM_Runner->getCurrentTargetLocal();
        }
    }
}
