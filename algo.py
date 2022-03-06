import cv2
import fn


if __name__ == '__main__':
    m_length = 400
    m_breadth = 250
    clearance = 5

    print('Validating points.....')
    p_valid = fn.valid_points(m_length, m_breadth, clearance)

    start = input("Provide Start coordinates  : x,y\n")
    start = (int(start.split(',')[0]), int(start.split(',')[1]))

    if fn.validity_check(start, p_valid, clearance):
        goal = input("Provide Goal coordinates : x,y\n")
        goal = (int(goal.split(',')[0]), int(goal.split(',')[1]))
        if fn.validity_check(goal, p_valid, clearance):
            print('generating')

            flag, map_parent, closed = fn.dijkstras_algorithm(
                start, goal, p_valid, clearance)

            if flag:
                print('Path Generated using dijikstras algorithm')
                path = fn.getPath(map_parent, start, goal)
                print(path)
                fn.animate(m_length, m_breadth, p_valid, closed, path)
                print('done!\nPress q to exit')
                cv2.waitKey(0)
            else:
                print('Could not generate the path')
                print(map_parent)
        else:
            print('Not a valid point, please try again')
    else:
        print('Not a valid point, please try again')
