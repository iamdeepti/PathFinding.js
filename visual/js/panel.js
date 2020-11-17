/**
 * The control panel.
 */
var Panel = {
  init: function () {
    var $algo = $("#algorithm_panel");

    $(".panel").draggable();
    $(".accordion").accordion({
      collapsible: false,
    });
    $(".option_label").click(function () {
      $(this).prev().click();
    });
    $("#hide_instructions").click(function () {
      $("#instructions_panel").slideUp();
    });
    $("#play_panel").css({
      top: $algo.offset().top + $algo.outerHeight() + 20,
    });
    $("#button2").attr("disabled", "disabled");
  },
  /**
   * Get the user selected path-finder.
   * TODO: clean up this messy code.
   */
  getFinder: function () {
    var finder,
      selected_header,
      heuristic,
      allowDiagonal,
      biDirectional,
      dontCrossCorners,
      weight,
      trackRecursion,
      timeLimit;

    selected_header = $(
      "#algorithm_panel " + ".ui-accordion-header[aria-selected=true]"
    ).attr("id");

    switch (selected_header) {
      case "breadthfirst_header":
        allowDiagonal =
          typeof $(
            "#breadthfirst_section " + ".allow_diagonal:checked"
          ).val() !== "undefined";
        biDirectional =
          typeof $(
            "#breadthfirst_section " + ".bi-directional:checked"
          ).val() !== "undefined";
        dontCrossCorners =
          typeof $(
            "#breadthfirst_section " + ".dont_cross_corners:checked"
          ).val() !== "undefined";
        if (biDirectional) {
          finder = new PF.BiBreadthFirstFinder({
            allowDiagonal: allowDiagonal,
            dontCrossCorners: dontCrossCorners,
          });
        } else {
          finder = new PF.BreadthFirstFinder({
            allowDiagonal: allowDiagonal,
            dontCrossCorners: dontCrossCorners,
          });
        }
        break;

      case "dijkstra_header":
        allowDiagonal =
          typeof $("#dijkstra_section " + ".allow_diagonal:checked").val() !==
          "undefined";
        biDirectional =
          typeof $("#dijkstra_section " + ".bi-directional:checked").val() !==
          "undefined";
        dontCrossCorners =
          typeof $(
            "#dijkstra_section " + ".dont_cross_corners:checked"
          ).val() !== "undefined";
        if (biDirectional) {
          finder = new PF.BiDijkstraFinder({
            allowDiagonal: allowDiagonal,
            dontCrossCorners: dontCrossCorners,
          });
        } else {
          finder = new PF.DijkstraFinder({
            allowDiagonal: allowDiagonal,
            dontCrossCorners: dontCrossCorners,
          });
        }
        break;
    }

    return finder;
  },
};
