template<typename _ROBOT_TYPE, typename _CELL_TYPE>
Cell<_ROBOT_TYPE, _CELL_TYPE>::Cell()
    : unif_(0., 1.)
{
    _ROBOT_TYPE()(obj_robot_);
    _CELL_TYPE()(obj_obstacle_, tf_obstacle_);
    ResetRNG();
}

template<typename _ROBOT_TYPE, typename _CELL_TYPE>
Cell<_ROBOT_TYPE, _CELL_TYPE>::Cell(std::vector<std::vector<Eigen::VectorXd>> inCoordinationDiagramPaths)
	: unif_(0., 1.), cdPaths(inCoordinationDiagramPaths)
{
	_ROBOT_TYPE()(obj_robot_);
	_CELL_TYPE()(obj_obstacle_, tf_obstacle_);
	ResetRNG();

	for (size_t i = 0; i < cdPaths.size(); i++)
	{
		std::vector<double> pathLengths;
		for (size_t x = 0; x < cdPaths[i].size()-1; x++)
		{
			double length = (cdPaths[i][x] - cdPaths[i][x+1]).norm();
			pathLengths.push_back(length);
		}
		cdPathLengths.push_back(pathLengths);
	}
}

template<typename _ROBOT_TYPE, typename _CELL_TYPE>
Eigen::VectorXd Cell<_ROBOT_TYPE, _CELL_TYPE>::NextRandomCspace()
{
    return _ROBOT_TYPE::Random(rng_, unif_);
}

template<typename _ROBOT_TYPE, typename _CELL_TYPE>
Eigen::VectorXd Cell<_ROBOT_TYPE, _CELL_TYPE>::NextRandomCfree()
{
    Eigen::VectorXd q;

    do
    {
        q = NextRandomCspace();
    } while (!CheckPosition(q));

    return q;
}

template<typename _ROBOT_TYPE, typename _CELL_TYPE>
void Cell<_ROBOT_TYPE, _CELL_TYPE>::ResetRNG()
{
    //uint64_t timeSeed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    uint64_t timeSeed = 0;
    std::seed_seq ss { uint32_t(timeSeed & 0xffffffff), uint32_t(timeSeed >> 32) };
    rng_.seed(ss);
}

template<typename _ROBOT_TYPE, typename _CELL_TYPE>
bool Cell<_ROBOT_TYPE, _CELL_TYPE>::JumpTo(const Eigen::VectorXd &q)
{
    if (!robot_.IsValidRange(q))
        return false;
    robot_.q() = q;
    return true;
}

template<typename _ROBOT_TYPE, typename _CELL_TYPE>
std::vector<Eigen::VectorXd> Cell<_ROBOT_TYPE, _CELL_TYPE>::convertToRealPosition(Eigen::VectorXd q)
{
	std::vector<Eigen::VectorXd> newQs;
	for (size_t robot = 0; robot < q.size(); robot++)
	{
		double lengthToGo = q[robot];
		for (size_t lengths = 0; lengths < cdPathLengths[robot].size(); lengths++)
		{
			lengthToGo -= cdPathLengths[robot][lengths];
			if (lengthToGo <= 0.0)
			{
				Eigen::VectorXd dir = cdPaths[robot][lengths] - cdPaths[robot][lengths + 1];
				dir.normalize();
				Eigen::VectorXd b = cdPaths[robot][lengths +1];
				Eigen::VectorXd newVector = b - dir * lengthToGo;			
				newQs.push_back(newVector);
				lengthToGo = 0.0;
				break;
			}
		}
		if (lengthToGo > 0.0)
		{
			int lengths = cdPathLengths[robot].size() - 1;
			Eigen::VectorXd dir = cdPaths[robot][lengths] - cdPaths[robot][lengths + 1];
			dir.normalize();
			Eigen::VectorXd b = cdPaths[robot][lengths + 1];
			Eigen::VectorXd newVector = b - dir * lengthToGo;
			newQs.push_back(newVector);
		}
	}
	return newQs;
}

template<typename _ROBOT_TYPE, typename _CELL_TYPE>
bool Cell<_ROBOT_TYPE, _CELL_TYPE>::CheckPosition(const Eigen::VectorXd &q)
{
	if (cdPaths.size() > 0)
	{
		std::vector<Eigen::VectorXd> realQ = convertToRealPosition(q);
		std::vector<fcl::Transform3f> transforms;
		for (size_t i = 0; i < realQ.size(); i++)
		{
			std::vector<fcl::Transform3f> cur = _ROBOT_TYPE::ForwardKinematic(realQ[i]);
			transforms.insert(transforms.end(), cur.begin(), cur.end());
		}
		tf_robot_ = transforms;
		if (tf_robot_.empty()) return false;
		for (size_t i = 0; i < 2; ++i)
		{
			/*
			// Test for collision with Obstacles
			for (size_t j = 0; j < obj_obstacle_.size(); ++j)
			{
			if (solver_.shapeIntersect(*obj_robot_[i], tf_robot_[i], *obj_obstacle_[j], tf_obstacle_[j], nullptr))
			return false;
			}
			*/

			// Test for collision with Other Robots
			for (size_t otherRobots = 0; otherRobots < 2; ++otherRobots)
			{
				if (otherRobots == i) continue;
				if (solver_.shapeIntersect(*obj_robot_[0], tf_robot_[i], *obj_robot_[0], tf_robot_[otherRobots], nullptr))
					return false;
			}
		}
	}
	else
	{
		tf_robot_ = _ROBOT_TYPE::ForwardKinematic(q);

		if (tf_robot_.empty()) return false;

		for (size_t i = 0; i < obj_robot_.size(); ++i)
		{
			/*
			// Test for collision with Obstacles
			for (size_t j = 0; j < obj_obstacle_.size(); ++j)
			{
				if (solver_.shapeIntersect(*obj_robot_[i], tf_robot_[i], *obj_obstacle_[j], tf_obstacle_[j], nullptr))
					return false;
			}
			*/

			// Test for collision with Other Robots
			for (size_t otherRobots = 0; otherRobots < obj_robot_.size(); ++otherRobots)
			{
				if (otherRobots == i) continue;
				if (solver_.shapeIntersect(*obj_robot_[i], tf_robot_[i], *obj_robot_[otherRobots], tf_robot_[otherRobots], nullptr))
					return false;
			}
		}
	}

    return true;
}

template<typename _ROBOT_TYPE, typename _CELL_TYPE>
bool Cell<_ROBOT_TYPE, _CELL_TYPE>::CheckMotion(const Eigen::VectorXd &from, const Eigen::VectorXd &to, float dx)
{
    Eigen::VectorXd segment(to - from), delta, current(from);
    int steps;
    bool cfree;

    delta = segment.normalized() * dx;
    steps = int(segment.norm() / dx);

    do
    {
        cfree = CheckPosition(current);
        current += delta;
    } while (cfree && --steps > 0);

    return cfree;
}

template<typename _ROBOT_TYPE, typename _CELL_TYPE>
bool Cell<_ROBOT_TYPE, _CELL_TYPE>::FirstContact(Eigen::VectorXd &Cfree, Eigen::VectorXd &Cobstacle, Eigen::VectorXd &from, const Eigen::VectorXd &to, float dx)
{
    Eigen::VectorXd segment(to - from), delta;
    int steps;

    delta = segment.normalized() * dx;
    steps = int(segment.norm() / dx);

    Cfree = from;
    Cobstacle = from;

    if (CheckPosition(Cfree))
    {
        do
        {
            Cobstacle += delta;
            if (!CheckPosition(Cobstacle)) return true;
            Cfree = Cobstacle;
        } while (--steps > 0);
    }

    Cobstacle = to;
    return false;
}

template<typename _ROBOT_TYPE, typename _CELL_TYPE>
bool Cell<_ROBOT_TYPE, _CELL_TYPE>::LastContact(Eigen::VectorXd &Cfree, Eigen::VectorXd &Cobstacle, Eigen::VectorXd &from, const Eigen::VectorXd &to, float dx)
{
    Eigen::VectorXd segment(to - from), delta;
    int steps;

    delta = segment.normalized() * dx;
    steps = int(segment.norm() / dx);

    Cfree = from;
    Cobstacle = from;

    if (!CheckPosition(Cfree))
    {
        do
        {
            Cobstacle += delta;
            if (CheckPosition(Cobstacle)) return true;
            Cfree = Cobstacle;
        } while (--steps > 0);
    }

    Cobstacle = to;
    return false;
}
