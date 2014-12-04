#ifndef AGENT_H_
#define AGENT_H_

/**
 * @namespace  obvious
 */
namespace obvious
{

/**
 * @class   Agent
 * @author  Stefan May
 * @date    23.10.2014
 */
class Agent
{

public:

  Agent(double x, double y);

  virtual ~Agent(void);

  void setPosition(double x, double y);

  void getPosition(double &x, double &y);

  void setOrientation(double theta);

  double getOrientation();

  unsigned int getID();

private:

  double _x;

  double _y;

  double _theta;

  unsigned int _ID;

};

} // end namespace

#endif
