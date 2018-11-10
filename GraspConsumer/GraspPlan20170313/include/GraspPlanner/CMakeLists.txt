set(hdrs GraspPlanner.h
    PARENT_SCOPE
    GraspControllerSVC_impl.h
    )

install(FILES ${hdrs} DESTINATION ${INC_INSTALL_DIR}/${PROJECT_NAME_LOWER}
    COMPONENT library)

